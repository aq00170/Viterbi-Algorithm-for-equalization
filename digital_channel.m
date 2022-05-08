function output_seq = digital_channel(input_seq, p)
%% digital_channel(input_seq, output_seq, p) receives an arbitrary N-bit sequence in its input
% and inverts each bit with a probability of P and does not change
% it with a probability of 1-P. In this way, this block acts like a
% digital channel whose inputs and outputs are bits and the probability
% This is binary stmmetric channel.
% of error in this channel is P
% input_seq must be an row vector
%%
errorbits = rand(size(input_seq)) < p; % toss some biased coins and make a logical index
output_seq = input_seq; % first perfectly copy
output_seq(errorbits) = 1 - output_seq(errorbits); % make 0 -> 1 and 1 -> 0
end