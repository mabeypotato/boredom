clear, clc, close all;

%% setup

grid.latlimitlower = 30;
grid.latlimitupper = 31;

grid.longlimitlower = 110;
grid.longlimitupper = 111;

grid.totallength = 3601 * 2;
grid.totalwidth = 3601 * 2;


%% import dted data

[grid_1, R] = readgeoraster("n30_e110_1arc_v3.dt2", "OutputType","double"); % bottom left
[grid_2, R] = readgeoraster("n30_e111_1arc_v3.dt2", "OutputType","double"); % bottom right
[grid_3, R] = readgeoraster("n31_e110_1arc_v3.dt2", "OutputType","double"); % top left
[grid_4, R] = readgeoraster("n31_e111_1arc_v3.dt2", "OutputType","double"); % top right

%% combine dteds

grid_total = zeros([grid.totallength, grid.totalwidth]);

grid_total(1:3601, 1:3601) = grid_3; % top left
grid_total(1:3601, 3602:7202) = grid_4; % top right
grid_total(3602:7202, 1:3601) = grid_2; % bottom left
grid_total(3602:7202, 3602:7202) = grid_1; % bottom right

%% plot them

latlim = R.LatitudeLimits;
lonlim = R.LongitudeLimits;
worldmap(latlim, lonlim);
geoshow(grid_total,R, "DisplayType", "surface");
demcmap(grid_total);