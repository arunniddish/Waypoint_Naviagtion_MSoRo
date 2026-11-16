%============================= MSoRo_IO =================
%
%  BLE interface to communicate and command MSoRo soft mobile robot
%  (Seeeduino XIAO, Nordic UART Service style firmware).
%
%  Requires: MATLAB Bluetooth Low Energy support (Instrument Control
%  Toolbox), Windows 10+/macOS with a BLE-capable adapter.
%
%  ============================= MSoRo_IO =================
classdef MSoRo_IO_Seeeduino < handle
    properties (Access = public)
        ble_props;            % name, timeout, UUIDs
        serial_data_rcvd;     % boolean flag: a complete line was received
        serial_data;          % cell array of received lines
        ble_device;           % ble object
        rx_char;              % MATLAB -> XIAO (write)
        tx_char;              % XIAO -> MATLAB (notify)
    end

    properties (Access = private)
        ble_open;
        rx_buffer;            % partial-line accumulator
        line_queue;           % complete lines waiting to be consumed
        user_cb;              % optional user callback (receives each line)
    end

    properties (Constant)
        % Nordic UART Service UUIDs -- must match the firmware
        SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
        RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  % write to robot
        TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  % notify from robot
        CHUNK_BYTES  = 20;     % safe payload for default 23-byte MTU
    end

    methods
        function this = MSoRo_IO_Seeeduino(~)
            this.ble_open = false;
            this.ble_props = struct('name', [], 'timeout', []);
            this.serial_data_rcvd = false;
            this.serial_data = {};
            this.rx_buffer = '';
            this.line_queue = {};
            this.user_cb = [];
        end

        % Connect to MSoRo over BLE
        %
        %   a_name:      advertised BLE name (e.g. "MSoRo") or MAC address
        %   a_timeout:   [optional] connect/scan timeout in sec. (default 30)
        %   a_cb_hdl:    [optional] user callback, called with each received line
        function connect( this, a_name, a_timeout, a_cb_hdl )
            assert( ~this.ble_open, ...
                '[MSoRo_IO::connect()] Already connected. Disconnect first.');
            if nargin < 3 || isempty(a_timeout), a_timeout = 30; end
            if nargin >= 4, this.user_cb = a_cb_hdl; end

            this.ble_props.name = a_name;
            this.ble_props.timeout = a_timeout;

            % Automatic retry until timeout (no manual re-running needed)
            t0 = tic;  last_err = [];
            while toc(t0) < a_timeout
                try
                    this.ble_device = ble(a_name);
                    break;
                catch err
                    last_err = err;
                    pause(1);
                end
            end
            assert( ~isempty(this.ble_device), ...
                '[MSoRo_IO::connect()] Could not connect to "%s": %s', ...
                a_name, getReport(last_err, 'basic'));

            this.rx_char = characteristic(this.ble_device, this.SERVICE_UUID, this.RX_UUID);
            this.tx_char = characteristic(this.ble_device, this.SERVICE_UUID, this.TX_UUID);

            this.rx_buffer = '';
            this.line_queue = {};
            this.tx_char.DataAvailableFcn = @(src, evt) this.msoro_ble_cb(src, evt);
            subscribe(this.tx_char, "notification");

            this.ble_open = true;
            fprintf('[MSoRo_IO::connect()] Connected to %s\n', a_name);
        end

        function disconnect( this )
            assert( this.ble_open, '[MSoRo_IO::disconnect()] Not connected.');
            try, unsubscribe(this.tx_char); catch, end
            this.tx_char = [];
            this.rx_char = [];
            this.ble_device = [];      % clearing the ble object drops the link
            this.ble_open = false;
        end

        % Issue MSoRo gait definition(s)
        %   a_transition_seq: cell array of transition-ID vectors
        %   a_gait_names:     cell array of gait names
        function define_gait( this, a_transition_seq, a_gait_names )
            assert( iscell(a_transition_seq), ...
                '[MSoRo_IO::define_gait()] Input must be cell array of vectors.');
            assert( this.ble_open, '[MSoRo_IO::define_gait()] Not connected.');

            for ii = 1:length(a_transition_seq)
                gait_def = sprintf('define %s', a_gait_names{ii});
                gait_def = [gait_def, sprintf(' %d', a_transition_seq{ii}), ' end '];
                this.line_queue = {};                       % drop stale replies
                this.send_string(gait_def);
                % Original code read two reply lines; wait for them (non-fatal)
                for k = 1:2
                    ln = this.wait_for_line(2);
                    if ~isempty(ln), fprintf('  <- %s\n', ln); end
                end
            end
        end

        % Start MSoRo gait
        %   a_gait_name:  name of gait
        %   a_num_cycles: number of cycles (default 1)
        function start_gait( this, a_gait_name, a_num_cycles )
            assert( this.ble_open, '[MSoRo_IO::start_gait()] Not connected.');
            if nargin < 3, a_num_cycles = 1; end

            % NOTE: your original sprintf passed 'end' as an extra argument, which
            % made MATLAB recycle the format ("start <gait> N start end ").
            % This version sends "start <gait> N end ". Adjust the firmware parser
            % if it relied on the old string.
            gait_cmd = sprintf('start %s %d end ', a_gait_name, a_num_cycles);
            this.send_string(gait_cmd);
            fprintf('Gait sequence %s initiated for %d times\n', a_gait_name, a_num_cycles);
        end

        % Block until a complete line arrives (or timeout). Returns '' on timeout.
        function line = wait_for_line( this, a_timeout )
            if nargin < 2, a_timeout = 5; end
            t0 = tic;
            while isempty(this.line_queue) && toc(t0) < a_timeout
                pause(0.01);          % lets the BLE callback run
            end
            if isempty(this.line_queue)
                line = '';
            else
                line = this.line_queue{1};
                this.line_queue(1) = [];
            end
        end

        % Write a string to the robot, split into BLE-sized chunks
        function send_string( this, str )
            bytes = uint8(str);
            n = numel(bytes);
            for k = 1:this.CHUNK_BYTES:n
                write(this.rx_char, bytes(k:min(k+this.CHUNK_BYTES-1, n)), "withoutresponse");
                pause(0.02);          % small gap so the XIAO's buffer keeps up
            end
        end

        % BLE notification callback (XIAO -> MATLAB). Reassembles lines.
        function msoro_ble_cb( this, src, ~ )
            while src.NumAvailableSamples > 0
                data = read(src, 'oldest');
                this.rx_buffer = [this.rx_buffer, char(data(:)')];
            end
            while true
                idx = find(this.rx_buffer == newline, 1);
                if isempty(idx), break; end
                line = strtrim(this.rx_buffer(1:idx-1));
                this.rx_buffer(1:idx) = [];
                if isempty(line), continue; end

                this.serial_data_rcvd = true;
                this.serial_data{end+1} = line;
                this.line_queue{end+1} = line;
                fprintf('[MSoRo_IO::msoro_ble_cb()] Data received: %s\n', line);
                if ~isempty(this.user_cb), this.user_cb(line); end
            end
        end
    end
end



%============================= MSoRo_IO =============================
%
%  BLE interface to communicate and command MSoRo soft mobile robot
%  (Seeeduino XIAO, Nordic UART Service style firmware).
%
%  Requires: MATLAB Bluetooth Low Energy support (Instrument Control
%  Toolbox), Windows 10+/macOS with a BLE-capable adapter.
%
%  ============================= MSoRo_IO ===========================

classdef MSoRo_IO_Seeeduino < handle
    properties (Access = public)
        all_ble_device;
        ble_props;            % name, timeout, UUIDs
        serial_data_rcvd;     % boolean flag: a complete line was received
        serial_data;          % cell array of received lines
        ble_device;           % ble object
        rx_char;              % MATLAB -> XIAO (write)
        tx_char;              % XIAO -> MATLAB (notify)
    end

    properties (Access = private)
        ble_open;
        rx_buffer;            % partial-line accumulator
        line_queue;           % complete lines waiting to be consumed
        user_cb;              % optional user callback (receives each line)
    end

    properties (Constant)
        %   % Nordic UART Service UUIDs -- must match the firmware
        %   SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
        %   RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  % write to robot
        %   TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  % notify from robot
        CHUNK_BYTES  = 20;     % safe payload for default 23-byte MTU
    end

    methods

        function this = MSoRo_IO_Seeeduino(~)
            this.all_ble_device = [];
            this.ble_open = false;
            this.ble_props = struct('name', [], 'timeout', []);
            this.serial_data_rcvd = false;
            this.serial_data = {};
            this.rx_buffer = '';
            this.line_queue = {};
            this.user_cb = [];
        end

        % Connect to MSoRo over BLE
        %
        %   a_name:        advertised BLE name (e.g. "MSoRo") or MAC address
        %   a_timeout:     [optional] connect/scan timeout in sec. (default 30)
        %   a_cb_hdl:      [optional] user callback, called with each received line
        %   a_ble_devices: All available ble devices
        function connect( this, a_name, a_timeout, a_ble_devices )
            assert( ~this.ble_open, ...
                '[MSoRo_IO::connect()] Already connected. Disconnect first.');

            % if nargin < 3 || isempty(a_timeout)
            %     a_timeout = 120; 
            % end
            % 
            % if nargin >= 4
            %     this.user_cb = a_cb_hdl;
            % end
            
            this.ble_props.name = a_name;
            this.ble_props.timeout = a_timeout;
            this.all_ble_device = a_ble_devices;

            % Automatically retry and connect to the respective device
            is_empty_or_missing = (this.all_ble_device.Name == "") | ismissing(this.all_ble_device.Name);
            this.all_ble_device.Name(is_empty_or_missing) = "None";

            idx_device = find(contains(this.all_ble_device.Name, this.ble_props.name));
            
            t0 = tic;
            while toc(t0) <= a_timeout 
                this.ble_device = ble(this.all_ble_device.Address(idx_device));
                pause(2);
                if this.ble_device.Connected
                    break;
                end
                disp("Not connected to the BLE device!");
            end

            assert(this.ble_device.Connected, 'Exiting...! BLE Device could not be found and it is not connected');

        end

        function define_gait( this, a_transition_seq, a_gait_names )
            pause(1);
            assert( strcmp(class(a_transition_seq), 'cell'), ...
                '[MSoRo_IO::define_gait()] Input must be cell array of vectors.');  % check input data format
            assert( this.ble_device.Connected, ...
                '[MSoRo_IO::define_gait()] No BLE device connected.');                % check serial port opened

            for ii = 1:length(a_transition_seq)
                %         gait_def = sprintf('%s%s%s',sprintf('define %s',a_gait_names{ii}),...
                %             sprintf('%3d',a_transition_seq{ii}),sprintf(' end '))
                gait_def = sprintf('%s',sprintf('define %s',a_gait_names{ii}));
                for i = 1:length(a_transition_seq{ii})
                    gait_def = sprintf('%s%s',gait_def, sprintf(' %d',a_transition_seq{ii}(i)));
                end
                gait_def = sprintf('%s%s', gait_def, ' end ')
                pause(1);
                write(this.ser_device,gait_def,"string");
                pause(2);
                data_recieve = strtrim(readline(this.ser_device))
                data_recieve = strtrim(readline(this.ser_device))
            end




            
            % Automatic retry until timeout (no manual re-running needed)
            t0 = tic;  last_err = [];
            while toc(t0) < a_timeout
                try
                    this.ble_device = ble(a_name);
                    break;
                catch err
                    last_err = err;
                    pause(1);
                end
            end
            assert( ~isempty(this.ble_device), ...
                '[MSoRo_IO::connect()] Could not connect to "%s": %s', ...
                a_name, getReport(last_err, 'basic'));

            this.rx_char = characteristic(this.ble_device, this.SERVICE_UUID, this.RX_UUID);
            this.tx_char = characteristic(this.ble_device, this.SERVICE_UUID, this.TX_UUID);

            this.rx_buffer = '';
            this.line_queue = {};
            this.tx_char.DataAvailableFcn = @(src, evt) this.msoro_ble_cb(src, evt);
            subscribe(this.tx_char, "notification");

            this.ble_open = true;
            fprintf('[MSoRo_IO::connect()] Connected to %s\n', a_name);
        end