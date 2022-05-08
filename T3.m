% in the name of god
% HW3
% Part1
% Convolutional Coding
% part2
% Equalization
% Alireza Qazavi
% 9913904
%% part1
clc;clear all;close all;
% Random Binary Sequence Generator
bit_uncoded = randi([0 1],1,10000);% create random stream of zeros and ones (binary)
trellis = poly2trellis(3,[6 7])
bit_coded = convenc(bit_uncoded,trellis);
subplot(3,1,1)
stem(bit_uncoded(1:10))% This makes the bitstream visible 
grid;
title('bit uncoded');
subplot(3,1,2)
stem(bit_coded(1:20))
title('bit coded');
grid
tbdepth = 34;
bit_decoded = vitdec(bit_coded,trellis,tbdepth,'trunc','hard');
subplot(3,1,3)
stem(bit_decoded(1:10))
title('bit decoded');grid

biterr(bit_uncoded,bit_decoded)
%% test of digital_channel
input_seq = randi([0,1],1,10000);
L = 0.01;
output_seq = digital_channel(input_seq, L);
u = biterr(input_seq,output_seq)/10000;
sprintf('bit error rate is %0.3f and P = %0.3f',u,L)
%%
figure;
subplot(5,1,1)
stem(bit_uncoded(1:10))% This makes the bitstream visible 
grid;
title('bit uncoded');
i = 2;
Error=[];
for L = [0.001,0.01,0.1,0.2]
   input_seq  = bit_coded;
   output_seq = digital_channel(input_seq, L);
   tbdepth = 34;
   bit_decoded = vitdec(output_seq,trellis,tbdepth,'trunc','hard');
   Error = [Error, biterr(bit_uncoded,bit_decoded)];
   subplot(5,1,i)
   stem(bit_decoded(1:10))
   grid;
   title(sprintf('bit decoded,This bit receive from ch. with p = %0.3f',L));
   i=i+1;
end
figure;
 bar([0.001,0.01,0.1,0.2],Error/10000)
 xlabel('Error Probability of CH.')
 ylabel('Error Probability with Viterbi')
 grid
 hold on
 plot([0.001,0.01,0.1,0.2],[0.001,0.01,0.1,0.2]);
 %% part2
 data_len = 1e4; % length of the data sequence
 random_inf_bits = randi([0,1],1,data_len);
 % BPSK Modulation
 BPSK_Seq = 2 * random_inf_bits -1;% change 1 to 1 and 0 to -1
 figure;
 subplot(2,1,1)
 stem(random_inf_bits(1:10));grid;title('Random Information Bits');ylabel('I_n')
 subplot(2,1,2)
 stem(BPSK_Seq(1:10));grid;title('BPSK Symbol');ylabel('a_n')
 h = [0.5 , 0.2 , 0.08]; %channel
 y = conv(BPSK_Seq,h);
 figure;
 subplot(3,1,1)
 stem(0:9,BPSK_Seq(1:10));grid;title('BPSK Symbol');ylabel('a_n')
 subplot(3,1,2)
 stem(0:numel(h)-1,h);grid;title('CIR');ylabel('h(t)');xlabel('t(sec)')
 subplot(3,1,3)
 stem(0:9,y(1:10));grid;title('Output of Channel');ylabel('y(t)');xlabel('t(sec)')
% Maximum likelihood sequence estimation using the Viterbi algorithm
% BPSK modulation, channel impairments: ISI (3 tap) and AWGN
% Note that this code works for a 3 tap channel only, if you wish to increase
% the number of channel taps, then you need to do minor changes in the code
chan_len = 3; % number of channel taps (DO NOT CHANGE THIS PARAMETER)
ber = [];q=1;
for SNR_dB = [10,20,30,40] % SNR per bit (in dB)
Sig_dB = pow2db(nnz(y));
Noise_dB = Sig_dB - SNR_dB;
h = [0.5 , 0.2 , 0.08]; %channel
y = conv(BPSK_Seq,h);
y = y + wgn(size(y,1),size(y,2),Noise_dB);
% SNR parameters
SNR = db2pow(SNR_dB);
% ------------------ RECEIVER----------------------------------------------
% steady state part of the received sequence
steady_state = y(chan_len:data_len);

% branch metrics for the Viterbi algorithm
% For storing Metric corresponding to each of the 8 transitions
L = data_len-chan_len+1;
branch_metric  = zeros(2^chan_len,L);
% For storing minimum distance for reaching each of the 8 states
node_value     = zeros(2^chan_len,L+1);
%Initialization of nodevalues
node_value(:,1)=0;
% according to trellis
branch_metric(1,:) = (steady_state-(-h(1)-h(2)-h(3))).^2;
branch_metric(2,:) = (steady_state-(+h(1)-h(2)-h(3))).^2;
branch_metric(3,:) = (steady_state-(-h(1)+h(2)-h(3))).^2;
branch_metric(4,:) = (steady_state-(+h(1)+h(2)-h(3))).^2;

branch_metric(5,:) = (steady_state-(-h(1)-h(2)+h(3))).^2;
branch_metric(6,:) = (steady_state-(+h(1)-h(2)+h(3))).^2;
branch_metric(7,:) = (steady_state-(-h(1)+h(2)+h(3))).^2;
branch_metric(8,:) = (steady_state-(+h(1)+h(2)+h(3))).^2;

for i = 1:L
%Minimum distance to reach state -1,-1,-1 at time i
node_value(1,i+1)=min(node_value(1,i)+branch_metric(1,i),...
    node_value(5,i)+branch_metric(1,i));
%Minimum distance to reach state -1,-1,1 at time i
node_value(2,i+1)=min(node_value(1,i)+branch_metric(2,i),...
    node_value(5,i)+branch_metric(2,i));
%Minimum distance to reach state -1,1,-1 at time i
node_value(3,i+1)=min(node_value(2,i)+branch_metric(3,i),...
    node_value(6,i)+branch_metric(3,i));
%Minimum distance to reach state -1,1,1 at time i
node_value(4,i+1)=min(node_value(2,i)+branch_metric(4,i),...
    node_value(6,i)+branch_metric(4,i));
%Minimum distance to reach state -1,1,-1 at time i
node_value(5,i+1)=min(node_value(3,i)+branch_metric(5,i),...
    node_value(7,i)+branch_metric(5,i));
%Minimum distance to reach state -1,1,1 at time i
node_value(6,i+1)=min(node_value(3,i)+branch_metric(6,i),...
    node_value(7,i)+branch_metric(6,i));
%Minimum distance to reach state -1,1,-1 at time i
node_value(7,i+1)=min(node_value(4,i)+branch_metric(7,i),...
    node_value(8,i)+branch_metric(7,i));
%Minimum distance to reach state -1,1,1 at time i
node_value(8,i+1)=min(node_value(4,i)+branch_metric(8,i),...
    node_value(8,i)+branch_metric(8,i));
end

%For the final stage pick the state corresponding to minimum weight
[~,Survivor_path]=min(node_value,[],1);
%Starting from the final stage using the state, node_value of previous stage
%and branch_metric, decoding the previous state and the corresponding Code bit
state = Survivor_path(end);

for j=L:-1:1
  [state,decoded_bit]=prev_stage(state,node_value(:,j),branch_metric(:,j));
  decoded_bit_final(j)=decoded_bit; %Storing the decoded bit in decode_bit_final vector
end

%-----------------------------------------------------------------------------------
% decoded_bit_final=[0,0,decoded_bit_final];
% figure
% subplot(2,1,1)
% stem(0:9,BPSK_Seq(9998-10+1:9998));grid;title('BPSK Symbol');ylabel('a_n')
% subplot(2,1,2)
% stem(0:9,decoded_bit_final(9998-10+1:9998));grid;title('Detected Symbol');ylabel('b_n')
decoded_bit_final = 0.5.*(decoded_bit_final+1);

% bit error rate (ignoring transient parts)
ber(q) = nnz(random_inf_bits(chan_len:data_len)-decoded_bit_final )...
    /(data_len-chan_len+1);
q = q + 1;
sprintf('Bit Error Rate is %0.0001d\n',ber)
end
figure;
semilogy([10,20,30,40],ber,'k-','linewidth',2.0)      %Plot for Simulated BER 
axis tight;grid;
legend('Simulated BER');
title('Viterbi decoder performance over AWGN channel for BPSK modulated symbols');
xlabel('SNR(dB)');ylabel('BER');
%-----------------------------------------------------------------------------------
% decoded_bit_final=double(decoded_bit_final);
% biterr(random_inf_bits,0.5*decoded_bit_final+0.5)/10000
% Viterbi algorithm
% dec_a=Viterbi_algorithm(data_len-chan_len+1,decoding_delay,branch_metric);

% bit error rate (ignoring transient parts)
% ber = nnz(a(chan_len:data_len-decoding_delay)-dec_a )/(data_len-chan_len+1-decoding_delay)