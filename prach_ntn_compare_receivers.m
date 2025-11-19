%%%%%%%%%%%%  MATLAB nrPRACHDetect vs SRSRAN MEX detector Test %%%%%%%%%%%%
% -------------------------------------------------------------------------------------------------------------------
% MATLAB nrPRACHDetect: https://www.mathworks.com/help/5g/ref/nrprachdetect.html
% SRSRAN MEX detector: https://docs.srsran.com/projects/project/en/latest/tutorials/source/matlab/source/index.html
% -------------------------------------------------------------------------------------------------------------------
% This code follows the following steps:
%   - Generate identical PRACH
%   - Apply a NTN-TDL channel in accordance to 
%     https://it.mathworks.com/help/satcom/ug/model-nr-ntn-channel.html
%   - Add AWGN with the desired SNR
%   - Compare detection probability and timing error of the two receivers
%
% Requirements:
%  - 5G Toolbox / SatCom Toolbox
%  - srsRAN-matlab: srsPRACHgenerator, srsPRACHdemodulator,
%                   srsMEX.phy.srsPRACHDetector

function prach_ntn_compare_receivers()
%% --------------------- Elevation angle sweep ---------------------------
% Define the set of elevation angles (degrees)
elevationAngles = 82:2:90;

%% --------------------- Simulation parameters --------------------------
numPRACHSlots = 100;
SNRdB = [-21 -16 -11 -6 -1];
threshold = [];
timeErrorTolerance_us = 2.55;

nRx = 2;

%% --------------------- Doppler pre-compensation control ----------------
% If true, the UE applies Doppler pre-compensation equal to -satDoppler.
% If false, no pre-compensation is applied.
enablePrecomp = true;

% Optional residual error in the pre-compensation (in Hz).
%   precompErrorHz = 0      -> perfect pre-compensation
%   precompErrorHz = 20     -> 20 Hz residual Doppler at the receiver
precompErrorHz = 0;

%% --------------------- PRACH configuration ----------------------------
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 15;
carrier.NSizeGrid = 25;

prach = nrPRACHConfig;
prach.FrequencyRange     = 'FR1';
prach.DuplexMode         = 'FDD';
prach.ConfigurationIndex = 27;
prach.SubcarrierSpacing  = 1.25;
prach.SequenceIndex      = 22;
prach.PreambleIndex      = 32;
prach.RestrictedSet      = 'UnrestrictedSet';
prach.FrequencyStart     = 0;

% Set ZCZ (NCS = 13)
try
    NCS = 13;
    switch prach.Format
        case {'0','1','2'}
            ncsTable = nrPRACHConfig.Tables.NCSFormat012;
            col = (string(ncsTable.Properties.VariableNames) == prach.RestrictedSet);
        case '3'
            ncsTable = nrPRACHConfig.Tables.NCSFormat3;
            col = (string(ncsTable.Properties.VariableNames) == prach.RestrictedSet);
        otherwise
            ncsTable = nrPRACHConfig.Tables.NCSFormatABC;
            col = contains(string(ncsTable.Properties.VariableNames), num2str(prach.LRA));
    end
    prach.ZeroCorrelationZone = ncsTable.ZeroCorrelationZone(ncsTable{:,col}==NCS);
catch
    % Keep default ZCZ if anything goes wrong
end

ofdmInfo = nrOFDMInfo(carrier);

%% --------------------- Arrays for results ------------------------------
nElev = numel(elevationAngles);
nSNR  = numel(SNRdB);

pDet_MATLAB     = zeros(nElev, nSNR);
pDet_SRS        = zeros(nElev, nSNR);
mae_MATLAB_us   = nan(nElev, nSNR);
mae_SRS_us      = nan(nElev, nSNR);

%% --------------------- SRS detector object -----------------------------
PRACHDetector = srsMEX.phy.srsPRACHDetector();

%% =======================================================================
%                      LOOP OVER ELEVATION ANGLES
% =======================================================================
for iEl = 1:nElev
    elev = elevationAngles(iEl);
    fprintf('\n\n================================ Elevation %d deg =====================================\n', elev);

    %% Compute satellite Doppler for this elevation
    common.CarrierFrequency  = 2e9;
    common.ElevationAngle    = elev;
    common.SatelliteAltitude = 300e3;
    common.MobileAltitude    = 0;
    common.MobileSpeed       = 3 * 1000/3600;
    common.Seed              = 73;

    satDoppler = dopplerShiftCircularOrbit( ...
        common.ElevationAngle, ...
        common.SatelliteAltitude, ...
        common.MobileAltitude, ...
        common.CarrierFrequency );

    c = physconst('lightspeed');
    mobileMaxDoppler = common.MobileSpeed * common.CarrierFrequency / c;

    % ----------------- Doppler pre-compensation setting -----------------
    % useFreqOffsetHz is the frequency offset applied at the receiver to
    % emulate UE pre-compensation (i.e., the signal is rotated by
    % exp(1j*2*pi*useFreqOffsetHz*t)).
    if enablePrecomp
        % Perfect pre-compensation if precompErrorHz = 0.
        % Positive precompErrorHz leaves a residual Doppler at the receiver.
        useFreqOffsetHz = -(satDoppler - precompErrorHz);
    else
        % No pre-compensation (full Doppler is present at the receiver).
        useFreqOffsetHz = 0;
    end
    % -------------------------------------------------------------------

    %% ===============================================================
    %                        SNR LOOP
    % ===============================================================
    for iSNR = 1:nSNR
        snr_dB = SNRdB(iSNR);

        detCountMat = 0;
        detCountSRS = 0;
        errAccMat = 0;   errAccSRS = 0;
        countMat  = 0;   countSRS  = 0;

        for nSlot = 0:numPRACHSlots-1

            % ----------- 1) PRACH generation (SRS) -------------------
            [txWave, gridset, genInfo] = ...
                srsLib.phy.upper.channel_processors.srsPRACHgenerator( ...
                    carrier, prach );

            prach.NPRACHSlot = genInfo.NPRACHSlot;

            % True timing offset (random in [0.1, 1.0] of the smallest ZCZ)
            trueDelay_s = (0.1 + 0.9*rand) / ...
                          (prach.SubcarrierSpacing * 1000 * 128);
            delaySamples = floor(trueDelay_s * gridset.Info.SampleRate);
            txWave = [zeros(delaySamples,1); txWave];

            % ----------- 2) NTN-TDL channel --------------------------
            ntnTDL = nrTDLChannel;
            ntnTDL.DelayProfile          = "NTN-TDL-C";
            ntnTDL.DelaySpread           = 30e-9;
            ntnTDL.TransmissionDirection = "Uplink";
            ntnTDL.MIMOCorrelation       = "Low";
            ntnTDL.Polarization          = "Co-Polar";
            ntnTDL.SampleRate            = gridset.Info.SampleRate;
            ntnTDL.MaximumDopplerShift   = mobileMaxDoppler;
            ntnTDL.SatelliteDopplerShift = satDoppler;
            ntnTDL.NumReceiveAntennas    = nRx;
            ntnTDL.NormalizePathGains    = true;
            ntnTDL.NormalizeChannelOutputs = true;
            ntnTDL.Seed = common.Seed;

            reset(ntnTDL);
            chInfo = info(ntnTDL);

            rx = ntnTDL([txWave; zeros(chInfo.MaximumChannelDelay,1)]);
            rx = rx((chInfo.ChannelFilterDelay+1):end, :);

            % ----------- 3) Optional Freq offset (pre-compensation) ---
            if useFreqOffsetHz ~= 0
                t = ((0:size(rx,1)-1)/ntnTDL.SampleRate).';
                rx = rx .* exp(1j*2*pi*useFreqOffsetHz*t);
            end

            % ----------- 4) Add AWGN ---------------------------------
            SNRlin = 10^(snr_dB/10);
            N0 = 1/sqrt(2.0 * nRx * double(ofdmInfo.Nfft) * SNRlin);
            rx = rx + N0 * complex(randn(size(rx)), randn(size(rx)));

            % ----------- 5a) MATLAB detector -------------------------
            [det, offs] = nrPRACHDetect(carrier, prach, rx, ...
                                        'DetectionThreshold', threshold);

            if numel(det)==1 && det==prach.PreambleIndex
                meas_s = offs(1)/ntnTDL.SampleRate;
                err_s  = abs(meas_s - trueDelay_s);
                if err_s <= timeErrorTolerance_us * 1e-6
                    detCountMat = detCountMat + 1;
                end
                errAccMat = errAccMat + err_s*1e6;  % accumulate error in microseconds
                countMat  = countMat  + 1;
            end

            % ----------- 5b) SRS detector ----------------------------
            grid_srs = srsLib.phy.lower.modulation.srsPRACHdemodulator( ...
                carrier, prach, gridset.Info, rx, genInfo);
            grid_srs = reshape(grid_srs, prach.LRA, prach.PRACHDuration, nRx);
            grid_srs = complex(single(real(grid_srs)), single(imag(grid_srs)));

            resSRS = PRACHDetector(prach, grid_srs);

            if ~isempty(resSRS.PreambleIndices)
                mask = (resSRS.PreambleIndices == prach.PreambleIndex);
                if sum(mask)==1
                    detCountSRS = detCountSRS + 1;
                    ta = resSRS.TimeAdvance(mask);
                    errAccSRS = errAccSRS + abs(ta - trueDelay_s)*1e6;
                    countSRS  = countSRS  + 1;
                end
            end

        end % slot loop

        % Store results
        pDet_MATLAB(iEl,iSNR) = detCountMat / numPRACHSlots;
        pDet_SRS(iEl,iSNR)    = detCountSRS / numPRACHSlots;

        if countMat>0
            mae_MATLAB_us(iEl,iSNR) = errAccMat / countMat;
        end
        if countSRS>0
            mae_SRS_us(iEl,iSNR) = errAccSRS / countSRS;
        end
        
        fprintf(['Elev %d°, SNR %+4.1f dB | pDet: MATLAB=%.3f SRS=%.3f ', ...
                 '| MAE(us): MATLAB=%.3f SRS=%.3f | precomp=%d, useFreqOffsetHz=%.1f Hz\n'], ...
            elev, snr_dB, ...
            pDet_MATLAB(iEl,iSNR), pDet_SRS(iEl,iSNR), ...
            mae_MATLAB_us(iEl,iSNR), mae_SRS_us(iEl,iSNR), ...
            enablePrecomp, useFreqOffsetHz);

    end % SNR loop
end % elevation loop

%% --------------------- Example plotting ------------------------------
figure('Name','Detection Probability (per elevation)');
for iEl = 1:nElev
    subplot(2,ceil(nElev/2),iEl);
    plot(SNRdB, pDet_MATLAB(iEl,:),'-o'); hold on;
    plot(SNRdB, pDet_SRS(iEl,:),'-x');
    title(sprintf('Elevation %d°', elevationAngles(iEl)));
    ylabel('pDet'); xlabel('SNR (dB)');
    grid on;
end
legend('MATLAB','SRSRAN');

figure('Name','MAE Timing (per elevation)');
for iEl = 1:nElev
    subplot(2,ceil(nElev/2),iEl);
    plot(SNRdB, mae_MATLAB_us(iEl,:),'-o'); hold on;
    plot(SNRdB, mae_SRS_us(iEl,:),'-x');
    title(sprintf('Elevation %d°', elevationAngles(iEl)));
    ylabel('MAE (us)'); xlabel('SNR (dB)');
    grid on;
end
legend('MATLAB','SRSRAN');

%% --------------------- Cleanup ----------------------------------------
release(PRACHDetector);

end
