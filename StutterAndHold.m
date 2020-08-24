classdef StutterAndHold < audioPlugin
    % STUTTERANDHOLD
    %
    %   Timbral tremolo effect through spectral freezing. Implements phase
    %   vocodeur, with voiced unvoiced detection, envelope following 
    %   and basilar compression. 
    %
    %   SAH = STUTTERANDHOLD returns audioPlugin object for this effect. To
    %   generate plugin for use outside of MATLAB:
    %
    %       validateAudioPlugin StutterAndHold
    %       generateAudioPlugin -au StutterAndHold
    %       generateAudioPlugin -vst StutterAndHold
    %
    % 
    %   Max Henry and Julian Vanasse 
    %   AES/Matlab student competition
    %   McGill University, Music Technology Area
    %   Summer 2020.
    
    properties (Access = public)
        
        % User-accessible parameters.
        
        FreezeToggle = true;
        AutoFreezeToggle = true;
        Refresh = false;
        VoicedThresh = false;
        
        FollowDepth = 0.0;
        RefreshRateHz = 4.0;
        DryWet = 1.0;
    end
    
    properties (Constant)
        
        % Interface parameters.
        
        PluginInterface = audioPluginInterface( ...
            audioPluginParameter('FreezeToggle',...
                'DisplayName','Freeze',...
                'Style', 'vtoggle'),...
            audioPluginParameter('AutoFreezeToggle',...
                'DisplayName','AutoFreeze On/Off',...
                'Style', 'vtoggle'),...
            audioPluginParameter('VoicedThresh',...
                'DisplayName','Vocal Mode',...
                'Style', 'vtoggle'),...
            audioPluginParameter('Refresh',...
                'DisplayName','Manual Refresh Trigger',...
                'Style', 'vtoggle'),...
            audioPluginParameter('RefreshRateHz',...
                'DisplayName','Auto-Freeze Refresh Rate',...
                'Mapping', {'lin', 0.5, 32},...
                'Style', 'vslider'),...
            audioPluginParameter('FollowDepth',...
                'DisplayName','Envelope Follower',...
                'Mapping', {'lin', 0, 1},...
                'Style', 'vslider'),...
            audioPluginParameter('DryWet',...
                'DisplayName','Dry/Wet Mix',...
                'Mapping', {'lin', 0, 1},...
                'Style', 'vslider'),...
            'InputChannels',1, ...
            'OutputChannels',1, ...
            'PluginName','Stutter+Hold', ...
            'VendorName','Infinite Series Audio', ...
            'VendorVersion','1.1.2', ...
            'UniqueId','SaH2', ...
            'BackgroundColor','w'); 
    end
    
    properties (Access = private)
        
        % Variables with initialized values.
        
        % Starting default sample-rate.
        Sr = 44100;
        
        % Analysis/Synthesis settings.
        HopSize = 1024;
        WindowSize = 4096;
        WindowGain = 1;
        
        % Auto-Freeze settings.
        RefreshRate = 8000;
        RefreshCount = 0;
        
        % Voiced/unvoiced crossfade increment.
        CrossfadeIncrement = 1/1024;
        
        % Envelope follower gain-adjust coefficient.
        Trim = 0.9;
        
        % Envelope-follower smoothing factor.
        EnvSmoothing = 800;
        
        % Wet signal dynamic-compression coefficient.
        Alpha = 1e-4;

        % Flags for refreshing freeze buffer.
        GrabFrame = false;
        GrabFB1 = false;
        
        % Voiced/Unvoiced flags.
        IsVoiced = false;
        PrevVoiced = false;
        CurVoiced = false;
        
    end
    
    
    properties (Access = private)
        
        % Variables to be initialized in the constructor.
        
        % Window vector.
        Window;
        
        % Crossfading.
        CrossfadeVal;
        EqPowCoef;
        
        % Buffers.
        AnBuf;
        SynBuf;
        EnvBuf;
        
        % Buffer dimensions.
        NumBufChans;
        BufferSize;
        NumFreqBins;
        
        % Buffer read/write positions.
        AnBufWritePos;
        AnBufReadPos;
        SynBufReadPos;
        
        % Spectral freeze assets.
        FreezeBuffer;
        FreezeCumulativePhase;
        PhaseAdvance;
        
        % Envelope compression.
        BasNorm;
        
    end
    
    methods
        
        function plugin = StutterAndHold
            
            % Constructor:
            
            % As many buffer channels as hop sizes in the buffer size.
            plugin.NumBufChans = ceil(plugin.WindowSize/plugin.HopSize);
            plugin.BufferSize = plugin.WindowSize;
            plugin.NumFreqBins = ceil(plugin.WindowSize / 2) + 1;
            
            % Analysis and synthesis buffers init, and amplitude env
            % buffer.
            plugin.AnBuf = zeros(plugin.BufferSize, plugin.NumBufChans);
            plugin.SynBuf = zeros(plugin.BufferSize, plugin.NumBufChans);
            plugin.EnvBuf = plugin.SynBuf;
            
            % Initialize Hann window.
            plugin.Window = hann(plugin.WindowSize, 'periodic') / sqrt(plugin.NumBufChans);
            
            % Read/write pointers:
            %
            % n.b: SynBufReadPos is identical to AnBufWritePos throughout
            % code; it is included as a redundant variable to aid in
            % human read/interpretability.
            
            plugin.AnBufWritePos = [1 (cumsum(ones(plugin.NumBufChans-1, 1))'*plugin.HopSize)+1];
            plugin.SynBufReadPos = plugin.AnBufWritePos;
            
            % Delayed dry read position to compensate for ana/syn latency.
            plugin.AnBufReadPos = 2;
            
            % Freeze assets.
            plugin.FreezeBuffer = zeros(plugin.BufferSize, 2);
            plugin.FreezeCumulativePhase = zeros(1, plugin.NumFreqBins);
            
            plugin.PhaseAdvance = zeros(1, plugin.NumFreqBins);
            plugin.PhaseAdvance(2:plugin.NumFreqBins) = ...
                (2*pi*plugin.HopSize)./(plugin.BufferSize./(1:(plugin.BufferSize/2)));
            
            % Crossfading initial value.
            plugin.CrossfadeVal = plugin.DryWet;
            plugin.EqPowCoef = 1/sqrt(2);
            
            % Envelope compression.
            plugin.updateAlpha();
        
        end
                

        function out = process(plugin,in)
            % Main processing loop:

            [NumSamples, NumChannels] = size(in);
            
            % Make output block.
            out = zeros(NumSamples, NumChannels);
            
            % Singe sample for wet signal calculation.
            PreOut = zeros(1, NumChannels);
            
            % Step through input samples:
            for n = 1:NumSamples
                
                % Update dry/wet crossfader value: if Freeze is off, set
                % fader value to dry-only.
                if ~plugin.FreezeToggle
                    plugin.CrossfadeVal = max(plugin.CrossfadeVal - ...
                        plugin.CrossfadeIncrement, 0.0);
                end
                
                % Auto-Freeze: is it time to refresh the frozen sound?
                if plugin.AutoFreezeToggle && (plugin.RefreshCount == 1)
                    plugin.GrabFrame = true;
                end
                
                % Clear previous wet-signal calculation.
                PreOut = PreOut*0;
                
                % Cycle through buffers:
                for b = 1:plugin.NumBufChans
                    
                    % Fill analysis buffers.
                    plugin.AnBuf(plugin.AnBufWritePos(b), b) = in(n);
                    
                    % At a full buffer, perform asychronous processing:
                    if plugin.AnBufWritePos(b) == plugin.BufferSize
                        
                        % Vocal Mode: Determine if voiced/unvoiced.
                        plugin.updateVoicedUnvoiced(b);
                        
                        % Envelope Follower: trace amplitude envelope.
                        plugin.EnvBuf(:, b) = ...
                            plugin.traceEnvelope(plugin.AnBuf(:, b));
                        
                        % Update freeze sound (constantly, or by trigger):
                        if ~plugin.FreezeToggle || plugin.GrabFrame

                            % Store last FreezeBuffer.
                            plugin.FreezeBuffer(:, 2) = ...
                                plugin.FreezeBuffer(:, 1);
                            
                            % Update and normalize current FreezeBuffer.
                            plugin.FreezeBuffer(:, 1) = plugin.safeNorm(plugin.AnBuf(:, b));
                            
                            % Phase vocodeur: compute and store phase.
                            ang = angle(fft(plugin.FreezeBuffer(:,1)));
                            plugin.FreezeCumulativePhase = ang(1:plugin.NumFreqBins);
                            
                            % Manual Refresh: keep on for two consecutive frames.
                            if plugin.GrabFB1
                                plugin.GrabFrame = false;
                                plugin.GrabFB1 = false;
                            end
                            
                            if plugin.GrabFrame
                                plugin.GrabFB1 = true;
                            end
                        
                        % If currently Freezing:
                        else
                            % Compress the traced amplitude envelope.
                            BasilEnv = plugin.basilarCompression(plugin.EnvBuf(:, b));
                            
                            % Adjust envelope to user-specified modulation depth.
                            %
                            %   n.b. The Trim value here is to attenuate the
                            %   no-modulation signal level. In practice, a
                            %   straight freeze signal *feels* louder than
                            %   an amplitude modulated freeze, even if the
                            %   latter gets to higher values than the former
                            %   *at points*. This is a basic psychoacoustic
                            %   principle at play.
                            
                            AmpEnv = plugin.FollowDepth * (BasilEnv - plugin.Trim) + plugin.Trim;
                            
                            % Calculate phase vocodeur signal and apply amplitude envelope.
                            plugin.SynBuf(:, b) =...
                                AmpEnv .* plugin.spectral() * sqrt(plugin.NumBufChans);
                        end
                        
                        % Apply synthesis window.
                        plugin.SynBuf(:, b) = power(plugin.WindowGain,2) * ...
                                                plugin.SynBuf(:,b) .* plugin.Window;
                    end
                    
                    % Sum buffers to update wet signal sample.
                    PreOut = PreOut + plugin.SynBuf(plugin.SynBufReadPos(b), b);
                end
                
                % Update crossfade value between wet and dry signal.
                %
                %   This is a DJ mixer-type idea: adding or subtracting 
                %   CrossFadeIncrement "slides" the mix in one direction or
                %   another; this avoids clipping from abrupt changes.
                %   
                %   This mechanism accounts for the vocal mode
                %   functionality, as well as the general dry-wet mix.
                
                if plugin.FreezeToggle   
                    if plugin.IsVoiced
                        plugin.CrossfadeVal = min(plugin.CrossfadeVal + ...
                            plugin.CrossfadeIncrement, plugin.DryWet);
                    else
                        plugin.CrossfadeVal = max(plugin.CrossfadeVal - ...
                            plugin.CrossfadeIncrement, 0.0);
                    end 
                end
                
                % Place calculated sample in "out":
                % Apply crossfade between wet signal and delay-compensated dry signal.
                out(n) = plugin.crossfade(PreOut, plugin.AnBuf(plugin.AnBufReadPos, 1));
                
                % Increment read/write pointers.
                plugin.AnBufWritePos = mod(plugin.AnBufWritePos, ...
                                        plugin.BufferSize) + 1;
                plugin.SynBufReadPos = mod(plugin.SynBufReadPos, ...
                                        plugin.BufferSize) + 1;
                                    
                plugin.AnBufReadPos = mod(plugin.AnBufReadPos, ...
                                        plugin.BufferSize) + 1;
                
                % Auto-Freeze: increment counter.
                plugin.RefreshCount = mod(plugin.RefreshCount, ...
                                        plugin.RefreshRate) + 1;
            end
           
        end
        
        function y = spectral(plugin)
            % Spectral processing with a phase vocodeur technique.

            % Fourier-transform and window current and last buffer.
            Xcurrent = fft(plugin.FreezeBuffer(:, 1) .* plugin.Window);
            Xlast   = fft(plugin.FreezeBuffer(:, 2) .* plugin.Window);
            
            % Truncate to positive frequencies.
            Xcurrent = Xcurrent(1:plugin.NumFreqBins);
            Xlast = Xlast(1:plugin.NumFreqBins);
            
            % Spectral magnitude.
            mX = abs(Xcurrent);
            
            % Phase vocodeur: (1) calculate and wrap phase advance.
            dp = angle(Xcurrent) - angle(Xlast) - plugin.PhaseAdvance';
            dp = plugin.piWrap(dp);
            
            % (2) Advance cumulative phase.
            plugin.FreezeCumulativePhase = plugin.piWrap(plugin.FreezeCumulativePhase + ...
                                             plugin.PhaseAdvance' + dp) ;
            
            % (3) Output spectrum.
            Y = mX .* exp(1i*plugin.FreezeCumulativePhase);
            
            % (4) Inverse transform.
            Y = [Y; conj(flip(Y(2:end-1)))]; % Reconstruct negative freqs.
            y = real(ifft(Y));
            
        end
        
        function Envelope = traceEnvelope(plugin, signal)
            % Trace amplitude envelope using analytic signal.
            
            Analytic = hilbert(signal);
            EnvelopeApprox = abs(Analytic);

            % Smooth with MA lowpass filter.
            M = plugin.EnvSmoothing;
            b = 1/M * ones(M, 1);
            Envelope = filter(b, 1, EnvelopeApprox);
            
            % n.b. this filtering adds a linear-phase floor(M/2) group delay.
            % In practice, the analysis audio is WINDOWSIZE samples ahead
            % of the output already, (this is compensated in the DRY out by
            % using the AnBufReadPos pointer to slightly delay the output),
            % so the delay imparted here is negligible. 
            %
            % After experimentation, we decided that adjusting for this
            % minute discrepency makes little difference in the qualitative
            % performance of the plugin.
        end
        
        function updateVoicedUnvoiced(plugin, b)
            % Vocal mode: Voiced/unvoiced detection. 
            % Uses a ratio of zero-crossing rate and frame-wise energy.
            
            Frame = plugin.safeNorm(plugin.AnBuf(:, b));
            
            ZeroCross = plugin.getZCR(Frame);
            FrameEnergy = plugin.getFrameEnergy(Frame); 
            
            plugin.CurVoiced = (FrameEnergy/ZeroCross > plugin.VoicedThresh * 0.5);            
            plugin.IsVoiced = (plugin.CurVoiced && plugin.PrevVoiced);
            plugin.PrevVoiced = plugin.CurVoiced;
        end
        
        
        function Mixture = crossfade(plugin, Sample1, Sample2)
            % Equal power crossfader between two signals.
            
            Mixture = plugin.EqPowCoef * (plugin.CrossfadeVal^0.5 * Sample1 + ...
                        (1 - plugin.CrossfadeVal)^0.5 * Sample2);
        end

        function CompressedEnv = basilarCompression(plugin, InputEnv)
            % Non-linear dynamic compression inspired by basilar membrane.

            CompressedEnv = log(2 ./ (1 + exp(-plugin.Alpha .* InputEnv)))/...
                plugin.BasNorm;
        end
        
        function set.Refresh(plugin, val)
            % When refesh changes value, grab new Freeze sound.
            
            plugin.Refresh = val;
            plugin.GrabFrame = true;
        end
        
        function set.FreezeToggle(plugin, val)
            % Apply compression when Freeze is on.
            
            plugin.FreezeToggle = val;
            plugin.updateAlpha();
        end
        
        function set.RefreshRateHz(plugin, val)
            % Auto-Freeze: convert refresh rate into number of samples.
            
            plugin.RefreshRateHz = val;
            plugin.RefreshRate = fix(plugin.Sr/val);
        end
        
        function plugin = updateAlpha(plugin)
           % This value determines compression curve.
           
           if plugin.FreezeToggle
               plugin.Alpha = 5.0;
           else
               plugin.Alpha = 1e-4;
           end
           
           % Update normalizing factor for the envelope.
           plugin.BasNorm = log(2 ./(1 + exp(-plugin.Alpha )));
        end
        
        function reset(plugin)
            plugin.Sr = getSampleRate(plugin);
            plugin.CrossfadeVal = plugin.DryWet;
            plugin.updateAlpha();
        end
        
    end
    
    methods (Static)
        function NormedFrame = safeNorm(Frame)
            % Safe normalization to avoid blowing up.
            maxVal = max(abs(Frame));
            
            if maxVal > 1e-4
                NormedFrame = Frame/maxVal;
            else
                NormedFrame = Frame;
            end
        end
        
        function y = piWrap(x)
            % Phase vocodeur: re-wrap phase.
            y = x - 2*pi*floor((x+pi)/(2*pi));
        end
        
        function ZeroCross = getZCR(Frame)
            % Vocal mode: calculate zero-crossing rate.
            A = sign(Frame);
            B = sign(circshift(Frame, 1));

            ZeroCross = sum(abs(A - B));
        end

        function FrameEnergy = getFrameEnergy(Frame)
            % Vocal mode: calculate frame energy.
            FrameEnergy = dot(Frame, Frame);
        end
    end
end
