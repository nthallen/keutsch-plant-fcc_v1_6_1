%%
cd C:\Users\nort\Documents\Documents\Exp\HCHO\FCC_V1_6_1\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM8');
%set(s,'BaudRate',57600);
set(s,'BaudRate',115200);
%%
% First check that the board is an FCC
BdID = read_subbus(s, 2);
if BdID ~= 7
  error('Expected BdID 7. Reported %d', BdID);
end
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);
fprintf(1, 'Attached to FCC S/N %d Build # %d\n', SerialNo, Build);
cmd_addr = 26; % Currently just for Plant FCC
%%
rm_obj = read_multi_prep([8,40,9,0]);
[vals,ack] = read_multi(s,rm_obj);
fprintf(1, 'Now figure out how to interpret the result\n');
%%
% Connect DAC 0 out to ADC 0 in
%  J2.A8 FLSET0 to J2.A2 FLOW0
%  J2.A12 or A5 (anagnd) to J2.A11 (FLOW0_RTN)
% A is the top connector. Pin 1 is in the upper right
test_dac_adc_loopback(s, 25, 0, 0, 0);
%%
test_dac_adc_loopback(s, 25, 1, 1, 0);
%%
test_dac_adc_loopback(s, 25, 2, 2, 0);
%%
test_dac_adc_loopback(s, 25, 3, 3, 0);
%%
% write_subbus(s, cmd_addr, 34); % Switch to single-ended 
test_dac_adc_loopback(s, 25, 0, 1, 1);
%%
% write_subbus(s, cmd_addr, 34); % Switch to single-ended negative lead
test_dac_adc_loopback(s, 25, 0, 1, 1, 0);


%% Test Commands
%
% Channel 0-3
% Mode 0: both off
%      1: Closed on
%      2: Open on
%      3: Open and closed on, but only for channel 3
channel = 3;
mode = 0;
write_subbus(s, cmd_addr, channel*4+mode);

%%
% Read and convert temp sensor values to volts
Vref = 2.5;
N = 100;
Volts = zeros(N,1);
for i=1:N
  navg = read_subbus(s, 33);
  TS0_Avg_LSW = read_subbus(s, 34);
  TS0_Avg_MSW = read_subbus(s, 35);
  TS0_Avg = cast(typecast(uint16([TS0_Avg_LSW,TS0_Avg_MSW]),'int32'),'double');
  TS0_Volts = TS0_Avg * Vref / (2^32); % Extra power of 2 here
  Volts(i) = TS0_Volts;
  fprintf(1, '%d: %.8f V\n', navg, TS0_Volts);
  pause(1);
end
figure;
plot(Volts, '.');

%%
% Read RH values
mr = read_multi_prep([hex2dec('25'), 1, hex2dec('28')]);
[vals,ack] = read_multi(s, mr);
if ack < 1
    fprintf(1, 'Ack %d\n', ack);
else
    sht_status = vals(1);
    sht_raw_temp = vals(2);
    sht_raw_rh = vals(3);
    sht_fcc_status = vals(4);
    if bitand(sht_status, 2^13)
        sht_heater = 'on';
    else
        sht_heater = 'off';
    end
    if bitand(sht_status, 2^15)
        sht_alert = 'Alert Pending:';
    else
        sht_alert = '';
    end
    if bitand(sht_status, 2^11)
        sht_rh_tracking_alert = 'RH Tracking Alert:';
    else
        sht_rh_tracking_alert = '';
    end
    if bitand(sht_status, 2^10)
        sht_t_tracking_alert = 'T Tracking Alert:';
    else
        sht_t_tracking_alert = '';
    end
    if bitand(sht_status, 2^4)
        sht_reset_detected = 'Reset Detected:';
    else
        sht_reset_detected = '';
    end
    if bitand(sht_status, 2^1)
        sht_cmd_status = 'Cmd Failure:';
    else
        sht_cmd_status = '';
    end
    if bitand(sht_status, 2^0)
        sht_crc_status = 'Write CRC Failure:';
    else
        sht_crc_status = '';
    end
    fprintf(1, 'SHT_Status:Htr %s:%s%s%s%s%s%s\n', sht_heater, sht_alert, ...
        sht_rh_tracking_alert, sht_t_tracking_alert, ...
        sht_reset_detected, sht_cmd_status, sht_crc_status);

    sht_t = -45+175*(sht_raw_temp/65535);
    sht_rh = 100 * sht_raw_rh / 65535;
    fprintf(1, 'SHT T: %.1f C  RH: %.1f%%\n', sht_t, sht_rh);
end
