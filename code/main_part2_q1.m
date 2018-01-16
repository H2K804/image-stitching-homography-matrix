%%
%% load images and match files for the first example
%%
close all;

matches = load('house_matches.txt'); 
%matches = load('library_matches.txt');
N = size(matches,1);

%-------------FIT FUNDAMENTAL------------

N = size(matches,1);
[F , residual] = fit_fundamental_1(matches, 1); % this is a function that you should write
L = F;


