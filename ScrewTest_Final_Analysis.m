clear all
clc
%% Importing file and variable creation

addpath('C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3');
% mkdir C:\Users\rhlax.DESKTOP-BIEGI88\OneDrive\AWEAR Lab\ScrewTest_Sep30\CutData
% mydir1 = 'C:\Users\rhlax.DESKTOP-BIEGI88\OneDrive\AWEAR Lab\ScrewTest_WithBrace_Dec10';
mydir = 'C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3';
        

vicon_files = dir(fullfile(mydir,'*.MARKER'));

length_vicon_folder = length(vicon_files);

t = 1;
g = 1;

for q = 1:length_vicon_folder
    mydir = 'C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3';
    
    vicon_files = dir(fullfile(mydir,'*.MARKER'));
    EMG_files = dir(fullfile(mydir,'*.EMG'));
    IMU_files = dir(fullfile(mydir,'*.IMU'));

    length_vicon_folder = length(vicon_files);
    length_EMG_folder = length(EMG_files);
    length_IMU_folder = length(IMU_files);

    vicon_file = vicon_files(q);
    EMG_file = EMG_files(q);
    IMU_file = IMU_files(q);

    % dlmread ignores the header

    raw_marker = dlmread(vicon_file.name,'\t',5,0);
    raw_EMG = dlmread(EMG_file.name,'',5,0);
    raw_IMU = dlmread(IMU_file.name,'',5,0);
%     raw_IMU = csvread(IMU_file.name,5);
    size_IMU = size(raw_IMU);
    IMU_col = size_IMU(2);
    
    freq = 100;
    time = 1/freq;
    %% Assigning each of the markers to a variable
    % Each variable contains x, y, and z position data

    if (q == 6) || (q == 7) || (q == 8) || (q == 9) || (q == 10) || (q == 11)
        
        % RSHO = [raw_marker(:,3) raw_marker(:,4) raw_marker(:,5)];
        % LSHO = [raw_marker(:,6) raw_marker(:,7) raw_marker(:,8)];
        % C7 = [raw_marker(:,9) raw_marker(:,10) raw_marker(:,11)];
        % CHST = [RawData(:,12) RawData(:,13) RawData(:,14)];
        % RUARMLAT = [raw_marker(:,15) raw_marker(:,16) raw_marker(:,17)];
        % RUARMMED = [raw_marker(:,18) raw_marker(:,19) raw_marker(:,20)];
        RELBLAT = [raw_marker(:,21) raw_marker(:,22) raw_marker(:,23)];
        % RLARMMED = [RawData(:,24) RawData(:,25) RawData(:,26)];
        % RLARMLAT = [RawData(:,27) RawData(:,28) RawData(:,29)];
        RELBMED = [raw_marker(:,30) raw_marker(:,31) raw_marker(:,32)];
        RWRLAT = [raw_marker(:,33) raw_marker(:,34) raw_marker(:,35)];
        % RFINMED = [RawData( :,36) RawData(:,37) RawData(:,38)];
        % RFIN = [RawData( :,39) RawData(:,40) RawData(:,41)];
        % RFINLAT = [raw_marker( :,42) raw_marker(:,43) raw_marker(:,44)];
        RWRMED = [raw_marker( :,45) raw_marker(:,46) raw_marker(:,47)];
        
    else
        
        RWRLAT = [raw_marker(:,3) raw_marker(:,4) raw_marker(:,5)];
        % RFINMED = [raw_marker(:,6) raw_marker(:,7) raw_marker(:,8)];
        % RFIN = [raw_marker(:,9) raw_marker(:,10) raw_marker(:,11)];
        % RFINLAT = [RawData(:,12) RawData(:,13) RawData(:,14)];
        RWRMED = [raw_marker(:,15) raw_marker(:,16) raw_marker(:,17)];
        % RLARMLAT = [raw_marker(:,18) raw_marker(:,19) raw_marker(:,20)];
        RELBMED = [raw_marker(:,21) raw_marker(:,22) raw_marker(:,23)];
        RELBLAT = [raw_marker(:,24) raw_marker(:,25) raw_marker(:,26)];
        % RLARMMED = [RawData(:,27) RawData(:,28) RawData(:,29)];
        % RUARMLAT = [RawData(:,30) RawData(:,31) RawData(:,32)];
        % RUARMMED = [RawData(:,33) RawData(:,34) RawData(:,35)];
        % RSHO = [RawData( :,36) RawData(:,37) RawData(:,38)];
        % LSHO = [RawData( :,39) RawData(:,40) RawData(:,41)];
        % C7 = [raw_marker( :,42) raw_marker(:,43) raw_marker(:,44)];
        % CHST = [raw_marker( :,45) raw_marker(:,46) raw_marker(:,47)];
    end
        


    %% Creating Vectors

    % Finding average wrist and elbow locations
    % Finding ave screwdriver location
    WristAve = (RWRMED + RWRLAT)/2;
    ElbowAve = (RELBMED + RELBLAT)/2;
    
%     SDTipAve = (SD3 + SD4)/2;
%     SDBaseAve = (SD2 + SD5)/2;

    % Making all relevant vectors with respect to the elbow
    % The elbow is our new "origin"

    RWRMED_new = RWRMED - ElbowAve;
    RWRLAT_new = RWRLAT - ElbowAve;
%     SD3_new = SD3 - SDBaseAve;
%     SD4_new = SD4 - SDBaseAve;

    RotAxis = WristAve - ElbowAve;  % Creating an axis between the wrist and elbow. This is what the wrist markers are rotating around
    RotAxis_sq = RotAxis.^2;
    RotAxis_mag = sqrt((RotAxis_sq(:,1) + RotAxis_sq(:,2) +  RotAxis_sq(:,3)));
    norm = RotAxis./RotAxis_mag;
    
%     SDRotAxis = SDTipAve - SDBaseAve;
%     SDRotAxis_sq = SDRotAxis.^2;
%     SDRotAxis_mag = sqrt((SDRotAxis_sq(:,1) + SDRotAxis_sq(:,2) +  SDRotAxis_sq(:,3)));
%     SDnorm = SDRotAxis./SDRotAxis_mag;

    %% Calculating Vectors

    % Finding the Components of the wrist vector parallel and
    % perpendicular to the rotation axis

    RWRMED_par = dot(norm,RWRMED_new,2).*norm;
    RWRLAT_par = dot(norm,RWRLAT_new,2).*norm;
%     SD3_par = dot(SDnorm,SD3_new,2).*SDnorm;
%     SD4_par = dot(SDnorm,SD4_new,2).*SDnorm;
%     
    
    RWRMED_perp = RWRMED_new - RWRMED_par;
    RWRLAT_perp = RWRLAT_new - RWRLAT_par;
%     SD3_perp = SD3_new - SD3_par;
%     SD4_perp = SD4_new - SD4_par;
    %% Setup for the equation theta = arctan(|WxV|/(W*V))

    % Calculating the cross product of adjacent perpendicular vectors
    
    for i = 1:length(RWRMED_perp)-1

        RWRMED_perp_cross(i,:) = cross(RWRMED_perp(i+1,:),RWRMED_perp(i,:)); 
        RWRLAT_perp_cross(i,:) = cross(RWRLAT_perp(i+1,:),RWRLAT_perp(i,:)); 
%         SD3_perp_cross(i,:) = cross(SD3_perp(i+1,:),SD3_perp(i,:)); 
%         SD4_perp_cross(i,:) = cross(SD4_perp(i+1,:),SD4_perp(i,:));
    end

    % Calculating the magnitude of the cross product vectors

    RWRMED_perp_cross_sq = RWRMED_perp_cross.^2;
    RWRMED_perp_cross_mag = sqrt(RWRMED_perp_cross_sq(:,1) + RWRMED_perp_cross_sq(:,2) +  RWRMED_perp_cross_sq(:,3));

    RWRLAT_perp_cross_sq = RWRLAT_perp_cross.^2;
    RWRLAT_perp_cross_mag = sqrt(RWRLAT_perp_cross_sq(:,1) + RWRLAT_perp_cross_sq(:,2) +  RWRLAT_perp_cross_sq(:,3));

%     SD3_perp_cross_sq = SD3_perp_cross.^2;
%     SD3_perp_cross_mag = sqrt(SD3_perp_cross_sq(:,1) + SD3_perp_cross_sq(:,2) +  SD3_perp_cross_sq(:,3));
%     
%     SD4_perp_cross_sq = SD4_perp_cross.^2;
%     SD4_perp_cross_mag = sqrt(SD4_perp_cross_sq(:,1) + SD4_perp_cross_sq(:,2) +  SD4_perp_cross_sq(:,3));
    
    % Calculating the dot product of the same adjacent perpendicular
    % vectors

    RWRMED_perp_dot = zeros(1,length(RWRMED_perp)-1);
    RWRLAT_perp_dot = zeros(1,length(RWRMED_perp)-1);

    for j = 1:length(RWRMED_perp)-1

        RWRMED_perp_dot(j) = dot(RWRMED_perp(j+1,:),RWRMED_perp(j,:));
        RWRLAT_perp_dot(j) = dot(RWRLAT_perp(j+1,:),RWRLAT_perp(j,:));
%         SD3_perp_dot(j) = dot(SD3_perp(j+1,:),SD3_perp(j,:));
%         SD4_perp_dot(j) = dot(SD4_perp(j+1,:),SD4_perp(j,:));

    end

    %% Calculating the angle "theta" between adjacent perpendicular vectors

    theta_med_rad = atan(RWRMED_perp_cross_mag./RWRMED_perp_dot.');
    theta_lat_rad = atan(RWRLAT_perp_cross_mag./RWRLAT_perp_dot.');

    theta_med_deg = atand(RWRMED_perp_cross_mag./RWRMED_perp_dot.');
    theta_lat_deg = atand(RWRLAT_perp_cross_mag./RWRLAT_perp_dot.');

%     theta_SD3_deg = atand(SD3_perp_cross_mag./SD3_perp_dot.');
%     theta_SD4_deg = atand(SD4_perp_cross_mag./SD4_perp_dot.');
    
    % Calculating the angular velocity "theta/second" of wrist rotation

%     theta_dot_med = zeros(1,length(theta_med_rad)-1);
%     theta_dot_lat = zeros(1,length(theta_med_rad)-1);

    for k = 1:length(theta_med_rad)-1

        theta_dot_med(k) = (theta_med_deg(k+1) - theta_med_deg(k))/time; 
        theta_dot_lat(k) = (theta_lat_deg(k+1) - theta_lat_deg(k))/time;
%         theta_dot_SD3(k) = (theta_SD3_deg(k+1) - theta_SD3_deg(k))/time; 
%         theta_dot_SD4(k) = (theta_SD4_deg(k+1) - theta_SD4_deg(k))/time; 

    end

    %% Plotting Theta

    frames1 = 1:length(theta_med_deg);
    frames2 = 1:length(theta_dot_med);

    theta_med_deg_wframe = [frames1' theta_med_deg];
    theta_lat_deg_wframe = [frames1' theta_lat_deg];
    theta_dot_deg_wframe = [frames2' theta_dot_med'];
%     theta_SD3_deg_wframe = [frames1' theta_SD3_deg];
%     theta_dot_SD3_deg_wframe = [frames2' theta_dot_SD3'];
    
%     figure(q)
% 
%     subplot(2,2,1)
%     plot(frames1,theta_med_deg)
%     hold on
%     yline(1)
%     title('Theta for Medial Wrist Marker')
%     % axis([0 250 -.1 .1])
%     % axis([0 5000 0 .1])
%     grid minor
%     grid on
% 
%     subplot(2,2,2)
%     plot(frames2,theta_dot_med)
%     hold on
%     yline(10)
%     title('Theta Dot for Medial Wrist Marker')
%     % axis([0 250 -1 1])
%     % axis([0 5000 -1.5 1.5])
%     grid minor
%     grid on
%     
%     subplot(2,2,3)
%     plot(frames1,theta_SD3_deg)
%     hold on
%     yline(1)
%     title('Theta for SD3')
%     % axis([0 250 -1 1])
%     % axis([0 5000 -1.5 1.5])
%     grid minor
%     grid on
%     
%     subplot(2,2,4)
%     plot(frames2,theta_dot_SD3)
%     hold on
%     yline(10)
%     title('Theta Dot for Medial Wrist Marker')
%     % axis([0 250 -1 1])
%     % axis([0 5000 -1.5 1.5])
%     grid minor
%     grid on
    
    %% Cutting Data
    
    x = 0;
    n = 0;
    
%     if (q == 12) || (q == 13) || (q == 14) || (q == 15) || (q == 16)
  
%         lower_cutoff = 1;
%         upper_cutoff = 2.5;
%     else 
            
        
%     end 
    
lower_cutoff = .5;   % This is the minimum value of theta that we are interested in
upper_cutoff = 1.25; % Any data sets that are above .5 but do not pass 1.25 are not reliable wrist rotations

    for c = 2:length(theta_med_deg_wframe) % This loop gets all of the data that goes above the lower cutoff

        if theta_med_deg_wframe(c,2) > lower_cutoff && theta_med_deg_wframe(c-1,2) < lower_cutoff
                n = n + 1;
                x = x + 1;
        elseif theta_med_deg_wframe(c,2) > lower_cutoff && theta_med_deg_wframe(c-1,2) > lower_cutoff
                n = n + 1;
                if theta_med_deg_wframe(1,2) > lower_cutoff
                    x = x + 1;
                end
        elseif theta_med_deg_wframe(c,2) < lower_cutoff && theta_med_deg_wframe(c-1,2) > lower_cutoff
                n = 0;
        end

           if n ~= 0
                theta_med_cut(n,x) = theta_med_deg_wframe(c,2);
           else 
           end
    end

    w = size(theta_med_cut);
    w = w(2);   % Here I am getting the number of columns in the theta cut matrix

    for col = 1:w   % This loop scans each stored column and if the data set never gets above the upper cutoff, 
                    % that column is made into zeros
        if max(theta_med_cut(:,col)) < upper_cutoff
            theta_med_cut(:,col) = 0;
        end
    end

    bad_columns = theta_med_cut(1,:) == 0;  % This finds all of the columns that are zeros and deletes them
    theta_med_cut(:,bad_columns) = [];
    theta_med_cut_col = nonzeros(theta_med_cut(:));

    find_marker_frames = ismember(theta_med_deg_wframe(:,2),theta_med_cut_col,'rows');  % This matches the correct 
                                                                                   % frames up with each theta value
    theta_cut_wframes = [theta_med_deg_wframe(find_marker_frames,1) theta_med_deg_wframe(find_marker_frames,2)];

    %% Flagging Each Wrist Motion in Cut Data

    % First loop flags the wrist motion when the subject is tightening the 
    % screw (identifies CW and CCW wrist rotations)
    % This is done looking at the gaps in the frame values

    go = true;

    for m = 1:length(theta_cut_wframes)-1

        if theta_cut_wframes(m,1) + 1 ~= theta_cut_wframes(m+1,1)

            if 50 < theta_cut_wframes(m+1,1) - theta_cut_wframes(m,1) &&  theta_cut_wframes(m+1,1) - theta_cut_wframes(m,1) < 249 

                   theta_cut_wframes(m+1,3) = 1; % Marking CW Rotation

            elseif theta_cut_wframes(m+1,1) - theta_cut_wframes(m,1) < 50 

                   theta_cut_wframes(m+1,3) = 2; % Marking CCW Rotation

            elseif theta_cut_wframes(m+1,1) - theta_cut_wframes(m,1) > 300

                   go = false;  % This is when the subject changes from tightening to loosening screws

            end 
        end
         if go == 0
            break
         end
    end

    % Second loop flags the data for when the subject is loosening the 
    % screw (identifies CW and CCW wrist rotations)
    % This is done looking at the gaps in the frame values

    for m = m:length(theta_cut_wframes)-1
        if theta_cut_wframes(m,1) - 1 ~= theta_cut_wframes(m-1,1)
           if 50 < theta_cut_wframes(m,1) - theta_cut_wframes(m-1,1) &&  theta_cut_wframes(m+1,1) - theta_cut_wframes(m,1) < 249 

               theta_cut_wframes(m,3) = 2; % Marking CW Rotation

           elseif theta_cut_wframes(m,1) - theta_cut_wframes(m-1,1) < 50 

               theta_cut_wframes(m,3) = 1; % Marking CCW Rotation

           end
        end
    end

    %% Cutting EMG and IMU data to match the cut theta data
    find_theta_dot_frames = ismember(theta_dot_deg_wframe(:,1),theta_cut_wframes(:,1));  % Finds the corresponding Theta Dot data using frame #
    theta_dot_cut = theta_dot_deg_wframe(find_theta_dot_frames,:);
    theta_dot_cut(:,3) = theta_cut_wframes(1:length(theta_dot_cut),3); % Including the flag in the theta dot data
    
    find_EMG_frames = ismember(raw_EMG(:,1),theta_dot_cut(:,1));  % Finds the corresponding EMG data using frame #
    EMG_cut = raw_EMG(find_EMG_frames,:);

    find_IMU_frames = ismember(raw_IMU(:,1),theta_dot_cut(:,1));  % Finds the corresponding IMU data using frame #
    IMU_cut = raw_IMU(find_IMU_frames,:);

%     find_SD_frames = ismember(raw_marker(:,1),theta_cut_wframes(:,1));
%     SD_cut = theta_SD3_deg(find_SD_frames,1);
%     theta_cut_wframes(:,4) = SD_cut;
    
%     find_theta_dot_SD3_frames = ismember(theta_dot_SD3_deg_wframe(:,1),theta_cut_wframes(:,1));  % Finds the corresponding Theta Dot data using frame #
%     theta_dot_SD3_cut = theta_dot_SD3_deg_wframe(find_theta_dot_SD3_frames,:);
%     theta_dot_cut(:,4) = theta_dot_SD3_cut(:,2);
    
%     bad_columns = raw_IMU(2,:) == 0;    %Eliminates all of the possible EMG and MAG columns in the IMU file
%     IMU_cut(:,bad_columns) = [];
    %% Creating Tables for Theta, EMG, and IMU and exporting them to .csv
    Theta_Table = array2table(theta_cut_wframes,'VariableNames',{'Frames','Theta (Deg)','Flag (1=CW 2=CCW)'});  % Creating the theta values table and then writing it to .csv     writetable(Theta_Table,'Theta_Cut_Values_05.csv');
%     Theta_Dot_Table = array2table(theta_dot_cut,'VariableNames',{'Frames','Theta Dot Values','Flag (1=CW 2=CCW)'});  % Creating the theta values table and then writing it to .csv     writetable(Theta_Table,'Theta_Cut_Values_05.csv');
    Theta_Dot_Table = array2table(theta_dot_cut,'VariableNames',{'Frames','Theta Dot (Deg/s)','Flag (1=CW 2=CCW)'});  % Creating the theta values table and then writing it to .csv     writetable(Theta_Table,'Theta_Cut_Values_05.csv');  
    EMG_Table = array2table(EMG_cut, 'VariableNames',{'Frame','Sub Frame','IM EMG1','IM EMG2','IM EMG4','IM EMG5','IM EMG6','IM EMG7','IM EMG8','IM EMG9','IM EMG10'}); % Creating the EMG values table and then writing it to .csv
    IMU_Table = array2table(IMU_cut,'VariableNames',{'Frame','Sub Frame','ACCX1','ACCY1','ACCZ1','GYROX1','GYROY1','GYROZ1','MAGX1','MAGY1','MAGZ1','ACCX2','ACCY2','ACCZ2','GYROX2','GYROY2','GYROZ2','MAGX2','MAGY2','MAGZ2','ACCX4','ACCY4','ACCZ4','GYROX4','GYROY4','GYROZ4','MAGX4','MAGY4','MAGZ4','ACCX5','ACCY5','ACCZ5','GYROX5','GYROY5','GYROZ5','MAGX5','MAGY5','MAGZ5','ACCX6','ACCY6','ACCZ6','GYROX6','GYROY6','GYROZ6','MAGX6','MAGY6','MAGZ6','ACCX7','ACCY7','ACCZ7','GYROX7','GYROY7','GYROZ7','MAGX7','MAGY7','MAGZ7','ACCX8','ACCY8','ACCZ8','GYROX8','GYROY8','GYROZ8','MAGX8','MAGY8','MAGZ8','ACCX9','ACCY9','ACCZ9','GYROX9','GYROY9','GYROZ9','MAGX9','MAGY9','MAGZ9','ACCX10','ACCY10','ACCZ10','GYROX10','GYROY10','GYROZ10','MAGX10','MAGY10','MAGZ10'});
    
     
    
    if (q == 1) || (q == 2) || (q == 3) || (q == 6) || (q == 7) || (q == 8) || (q == 12)
        
        writetable(Theta_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\Theta_Cut_Bulb_0' num2str(t) '.csv']);
        writetable(Theta_Dot_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\Theta_Dot_Cut_Bulb_0' num2str(t) '.csv']);
        writetable(IMU_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\IMU_Cut_Bulb_0' num2str(t) '.csv']);
        writetable(EMG_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\EMG_Cut_Bulb_0' num2str(t) '.csv']);
        t = t + 1;
        
    else
        
        writetable(Theta_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\Theta_Cut_SD_0' num2str(g) '.csv']);
        writetable(Theta_Dot_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\Theta_Dot_Cut_SD_0' num2str(g) '.csv']);
        writetable(IMU_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\IMU_Cut_SD_0' num2str(g) '.csv']);
        writetable(EMG_Table,['C:\Users\rhlax.DESKTOP-BIEGI88\Documents\AWEAR (Desktop)\Screw and Bulb Feb 3\CutData\EMG_Cut_SD_0' num2str(g) '.csv']);
        g = g + 1;
    
    end
    
    clearvars -except t g 
end

