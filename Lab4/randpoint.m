function [P] = randpoint(dims)
%UNTITLED3 Summary of this function goes here
%   Dims (dim x 2): size of the boundary and the start/ends of each boundary
[row,col]=size(dims);
P=[];
    for i=1:row
        P(i)=rand(1)*(abs(dims(i,1)-dims(i,2)))+dims(i,1);
    end
end

