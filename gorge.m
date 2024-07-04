clear, clc, close all;

%% import dted data and plot 

[grid_3, R3] = readgeoraster("n30_e110_1arc_v3.dt2", "OutputType","double"); % bottom left
[grid_4, R4] = readgeoraster("n30_e111_1arc_v3.dt2", "OutputType","double"); % bottom right
[grid_2, R2] = readgeoraster("n31_e110_1arc_v3.dt2", "OutputType","double"); % top left
[grid_1, R1] = readgeoraster("n31_e111_1arc_v3.dt2", "OutputType","double"); % top right

[big_grid, RA] = mergetiles(grid_1, R1, grid_2, R2, grid_3, R3, grid_4, R4);
%%
figure;
worldmap(big_grid, RA);
geoshow(big_grid, RA, 'DisplayType', 'surface')
demcmap(big_grid);
title('a really cool place for a B-21 training flight');