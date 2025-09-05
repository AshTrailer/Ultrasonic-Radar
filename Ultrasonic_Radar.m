clc; clear; close all;

%% Serial port initialization
% Create a serialport object to communicate with Arduino.
% Data is expected to be streamed as formatted text from the Arduino sketch.
s = serialport("COM6", 115200);
configureTerminator(s,"LF");

%% Radar configuration parameters
radarRadius = 75;        % Maximum display radius (cm)
markerSize = 3;          % Size of scatter points

fps = 30;                % Target frame refresh rate
fadeTimePoint = 10;      % Lifetime of an echo point before disappearing (s)
fadeTimeLine = 2;        % Lifetime of the fading radar sweep (s)
fadeSpeed = 1 / (fadeTimePoint * fps);      % Fading speed for echo points
fadeSpeedLine = 1 / (fadeTimeLine * fps);   % Fading speed for sweep trails

width = 2;               % Angular width of the active sweep line (degrees)
fadeWidth = 30;          % Angular range over which sweep trails are visible

%% Radar background initialization
% Draws the radar background: circular boundary, grid circles,
% radial lines with angular labels, and range markings.
figure(1); set(gcf,'Color','k'); axis equal; hold on;
xlim([-radarRadius radarRadius]); ylim([-radarRadius radarRadius]);
axis off; set(gca,'Color','k');

% Radar circular boundary
thetaFill = linspace(0,2*pi,360);
fill(radarRadius*cos(thetaFill), radarRadius*sin(thetaFill), ...
    [0 0.3 0], 'FaceAlpha',0.3,'EdgeColor','none');

% Concentric range circles
thetaCircle = linspace(0,2*pi,360);
for rFrac = [0.25 0.5 0.75 1.0]
    r = radarRadius * rFrac;
    plot(r*cos(thetaCircle), r*sin(thetaCircle), ...
        'Color',[0 0.3 0],'LineWidth',1);
end

% Angular grid lines and degree labels
for angleDeg = 0:30:330
    x = radarRadius*cosd(angleDeg);
    y = radarRadius*sind(angleDeg);
    text(x, y, sprintf('%.0f°', angleDeg), 'Color',[0 0.5 0], ...
        'FontSize',10,'HorizontalAlignment','center','VerticalAlignment','middle');
    plot([0 x],[0 y],'Color',[0 0.2 0],'LineWidth',1);
end

% Range labels on the x-axis
for rFrac = [0.25 0.5 0.75 1.0]
    r = radarRadius*rFrac;
    text(r,0,sprintf('%.0f',r),'Color',[0 0.5 0],'FontSize',10, ...
        'HorizontalAlignment','center','VerticalAlignment','top');
end

%% Echo point initialization
% Pre-allocate scatter plot to store echo points.
maxEchoes = 500;
xEcho = nan(1,maxEchoes); 
yEcho = nan(1,maxEchoes);
alphaEcho = zeros(1,maxEchoes);
hScatter = scatter(xEcho, yEcho, markerSize^2, 'filled', ...
    'MarkerFaceColor',[0 0.9 0],'MarkerEdgeColor','none', ...
    'MarkerFaceAlpha','flat','AlphaData',alphaEcho);

%% Radar sweep trail initialization
% Store past sweep patches for fading effect.
scanHistory = struct('theta',{},'patchHandle',{});

scanAngle = 0;
while ishandle(gca)
    % -------- Serial data acquisition --------
    % Expecting data format: [hh:mm:ss,angle,distance]
    if s.NumBytesAvailable > 0
        dataLine = readline(s);
        dataLine = strrep(dataLine,'[',''); 
        dataLine = strrep(dataLine,']','');
        tokens = strsplit(dataLine,',');
        if numel(tokens)==3
            angle = str2double(tokens{2});
            dist = tokens{3};
            if strcmp(dist,'OutRange'), continue; end
            dist = str2double(dist);
            scanAngle = mod(angle,360);

            % Convert polar coordinates (angle, distance) to Cartesian
            r = min(dist, radarRadius);
            x = r*cosd(scanAngle);
            y = r*sind(scanAngle);

            % Insert new echo point into scatter data
            idx = find(alphaEcho==0,1); 
            if isempty(idx), idx=1; end
            xEcho(idx)=x; 
            yEcho(idx)=y; 
            alphaEcho(idx)=1;
        end
    end

    % -------- Echo fading update --------
    % Gradually reduce transparency of old echo points.
    alphaEcho = max(0, alphaEcho - fadeSpeed);
    set(hScatter,'XData',xEcho,'YData',yEcho,'AlphaData',alphaEcho);

    % -------- Current radar sweep line --------
    % Draw active sweep as a bright green sector.
    thetaStart = scanAngle - width;
    thetaEnd = scanAngle + width;
    thetaPatch = deg2rad(linspace(thetaStart, thetaEnd, 20));
    xPatch = [0 radarRadius*cos(thetaPatch) 0];
    yPatch = [0 radarRadius*sin(thetaPatch) 0];
    hPatch = fill(xPatch, yPatch, [0 0.9 0], ...
        'FaceAlpha',0.8, 'EdgeColor','none');

    % Keep echo points rendered above sweep lines
    uistack(hScatter,'top');

    % -------- Sweep trail fading --------
    % Store the sweep line and gradually fade older trails.
    scanHistory(end+1).theta = scanAngle;
    scanHistory(end).patchHandle = hPatch;

    for k = length(scanHistory):-1:1
        oldPatch = scanHistory(k).patchHandle;
        currentAlpha = get(oldPatch,'FaceAlpha');

        % The most recent sweep line stays bright
        if k==length(scanHistory)
            alphaDecay = 0; 
        else
            alphaDecay = fadeSpeedLine;
        end
        currentAlpha = currentAlpha - alphaDecay;

        % Remove trails once fully faded
        if currentAlpha <= 0
            delete(oldPatch);
            scanHistory(k) = [];
        else
            % Older trails are drawn in darker green
            if k < length(scanHistory)
                set(oldPatch,'FaceColor',[0 0.4 0]); 
            end
            set(oldPatch,'FaceAlpha',currentAlpha);
        end
    end

    % -------- Frame update --------
    drawnow limitrate;
end
