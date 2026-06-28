clear

% Load the data -> .csv file was converted into .mat
load("data\Waveplate_A.mat")
X = waveplate_A(:,1);   % angles in deg
Y = waveplate_A(:,2);   % coincident counts


% Define fit options and type and perform the fit
wp_opt = fitoptions('Method','NonlinearLeastSquares',...
                    'Lower',[0,0,0],...
                    'Upper',[Inf,2*pi, min(Y)],...
                    'StartPoint',[max(Y), 0, 0]);

wp_typedef = fittype("a*cos(2*x+b).^2+c",...
                     dependent="y",independent="x",...
                     coefficients=["a" "b" "c"], options=wp_opt);

wp_fit = fit(X*pi/180,Y,wp_typedef);


% Compute the fitted curve with minimum and maximum
x_val = 0:0.5:135;
y_fit = wp_fit.a*cos(2*x_val*pi/180+wp_fit.b).^2+wp_fit.c;

[~,alpha_min_idx] = min(y_fit);
alpha_min = x_val(alpha_min_idx);
disp("Minimum of HWP_A at:  "+alpha_min)

% Plot the curve and datapoints
f1 = figure(1);
plot(X,Y,"o",LineWidth=1.5)
hold on
plot(x_val,y_fit,LineWidth=1.5)
hold off
grid on
xlim([0,135])
xlabel("Angle of HWP_A [deg]") 
ylabel("Conicidence count")
legend("Measured", "Fit")
