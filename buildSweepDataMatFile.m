% to be filled by user
% it is the ID as displayed in the Capture iPadOS app
% aka the number in a blue circle
localIds = [2 3 4 5 6 7 8 9 10 11 12 13 14 15, ...
            16 20 21 22 23 24 25 27 28 29 30 31, ...
            32 33 35 36 37];

%% Fill in the sweep data structure
assert(size(localIds,2) == size(in,1));
in = jsondecode(fileread('sweepData.json'));
for i=1:size(in,1)
    out(i).uuid = in(i).uuid;
    out(i).localId = localIds(i);
    out(i).position = [in(i).position.x; in(i).position.y; in(i).position.z];
    out(i).rotation = [in(i).rotation.x in(i).rotation.y in(i).rotation.z];
end

sweepData = out;

%% Save the sweep data
save('sweepData.mat', 'sweepData');