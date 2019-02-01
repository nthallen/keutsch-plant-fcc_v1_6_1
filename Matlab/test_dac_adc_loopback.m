function test_dac_adc_loopback(s, N, dac_channel, adc_channel, single_ended, positive)
  % test_dac_adc_loopback(s, N, dac_channel, adc_channel, single_ended, positive)

  %N = 25;
  %dac_channel = 0;
  %adc_channel = 1;
  cmd_addr = 26; % 26 is for Plant FCC, 24 for HCHO FCC
  if single_ended == 0
    write_subbus(s, cmd_addr, 35); % Switch to differential
    adc_mode = 'Differential';
  else
    write_subbus(s, cmd_addr, 34); % Switch to single-ended 
    if nargin < 6 || positive > 0
      write_subbus(s, cmd_addr, 36);
      adc_mode = 'Single-ended';
    else
      write_subbus(s, cmd_addr, 37);
      adc_mode = 'Single-ended negative lead';
    end
  end
  readback = zeros(N,1);
  readback2 = zeros(N,1);
  adc = zeros(N,1);
  adc2 = zeros(N,1);
  setpoints = floor(linspace(0,65535,N))';
  for i = 1:N
    write_subbus(s, 20+dac_channel, setpoints(i));
    readback(i) = read_subbus(s, 20+dac_channel);
    pause(0.1);
    adc(i) = read_subbus(s, 16+adc_channel);
  end
  for i = N:-1:1
    write_subbus(s, 20+dac_channel, setpoints(i));
    readback2(i) = read_subbus(s, 20+dac_channel);
    pause(0.2);
    adc2(i) = read_subbus(s, 16+adc_channel);
  end
  if any(setpoints ~= readback) || any(setpoints ~= readback2)
    error('Not all setpoints read back correctly');
  end
  adcS = adc - (adc>=32768)*65536;
  adcV = adcS * 6.144 / 32768;
  adc2S = adc2 - (adc2>=32768)*65536;
  adc2V = adc2S * 6.144 / 32768;
  dacVref = 2.490;
  dacV = setpoints*2*dacVref/65536;
  %
  f = figure;
  pos = get(f,'position');
  pos(1) = 20;
  pos(3) = 1200;
  set(f,'position',pos);
  ax = [ subplot(1,2,1), subplot(1,2,2) ];
  %
  plot(ax(1),dacV,dacV,dacV,adcV,'.',dacV,adc2V,'+');
  title(ax(1),sprintf('Channel %d/%d Loopback %s', dac_channel, ...
    adc_channel, adc_mode));
  xlabel(ax(1),'Setpoint V');
  ylabel(ax(1),'Readback V');
  %
  plot(ax(2),dacV,adcV-dacV,'.',dacV,adc2V-dacV,'+');
  title(ax(2),sprintf('Channel %d/%d Loopback Residual %s', ...
    dac_channel, adc_channel, adc_mode));
  xlabel(ax(2),'Setpoint V');
  ylabel(ax(2),'Error V');
  
