%------------------------------------------------------------------------------
% Create 'cube' rigidbody with one free joint (6-DoF)
% must have newest version of matlab!
%------------------------------------------------------------------------------
cube = rigidBodyTree('DataFormat','column');
body1 = rigidBody('body1');
joint1 = rigidBodyJoint('joint1','floating');
body1.Joint = joint1;
addVisual(body1, 'box', [0.25 0.25 0.25]);
addBody(cube,body1,cube.BaseName);

pos_init = [0 0 0]';
orient_init = so3(eye(3));
q_init = [quat(orient_init)'; pos_init];

% Use this to change example output:
% 1-5 is for Part A, 6-7 for Part B, 8 for Part C
whichExample = 5;


%------------------------------------------------------------------------------
% Example 1:
% Cube in the 'home' configuration:
%------------------------------------------------------------------------------
if whichExample == 1

  position = [0 0 0]';
  orientation = so3(eye(3));
  q = [quat(orientation)'; position];
  note = [ ...
    'Example 1: Cube in the home position\n', ...
    'JointPosition: ', sprintf('%.2f ', q')];

% -----------------------------------------------------------------------------
% Example 2:
% Cube rotated by 45 deg about z-space axis
% -----------------------------------------------------------------------------

elseif whichExample == 2

  % z- Rotation Matrix using anonymous function
  Rz =@(a) [cos(a) -sin(a) 0;
            sin(a) cos(a) 0;
            0 0 1];


  phiz = pi/8;
  position = [0 0 0]';
  orientation = so3(Rz(phiz));
  q = [quat(orientation)'; position];

  note = [ ...
    'Example 2: Cube rotated by 45 deg about z-space axis\n', ...
    'JointPosition: ', sprintf('%.2f ', q')];

% ----------------------------------------------------------------------------
% Part A. Generate figures for the following orientations
elseif whichExample == 3

  % 45 degree for Z axis
  % z- Rotation Matrix using anonymous function
  Rz =@(a) [cos(a) -sin(a) 0;
            sin(a) cos(a) 0;
            0 0 1];


  phiz = pi/4;
  position = [0 0 0]';
  orientation = so3(Rz(phiz));
  q = [quat(orientation)'; position];

  note = [ ...
    'Example 3: Cube rotated by 45 deg about z-space axis\n', ...
    'JointPosition: ', sprintf('%.2f ', q')];

elseif whichExample == 4

  % 45 degree for Y axis
  % y- Rotation Matrix using anonymous function
  Ry =@(a) [cos(a) 0 sin(a);
            0 1 0;
            -sin(a) 0 cos(a)];


  phiz = pi/4;
  position = [0 0 0]';
  orientation = so3(Ry(phiz));
  q = [quat(orientation)'; position];

  note = [ ...
    'Example 4: Cube rotated by 45 deg about y-space axis\n', ...
    'JointPosition: ', sprintf('%.2f ', q')];

elseif whichExample == 5

  % 45 degree for X axis
  % x- Rotation Matrix using anonymous function
  Rx =@(a) [1 0 0;
            0 cos(a) -sin(a);
            0 sin(a) cos(a)];

  phiz = pi/4;
  position = [0 0 0]';
  orientation = so3(Rx(phiz));
  q = [quat(orientation)'; position];

  note = [ ...
    'Example 5: Cube rotated by 45 deg about x-space axis\n', ...
    'JointPosition: ', sprintf('%.2f ', q')];

% end of Part A
% ----------------------------------------------------------------------------


% ----------------------------------------------------------------------------
% Part b. Successive Rotations

% part_b_1
elseif whichExample == 6

  % Space frame, successive rotations for z<-y<-x with 45 degree for each one
  Rx =@(a) [1 0 0;
            0 cos(a) -sin(a);
            0 sin(a) cos(a)];

  Ry =@(a) [cos(a) 0 sin(a);
            0 1 0;
            -sin(a) 0 cos(a)];

  Rz =@(a) [cos(a) -sin(a) 0;
            sin(a) cos(a) 0;
            0 0 1];

  Rsucc =@(a) Rz(a) * Ry(a) * Rx(a);

  phiz = pi/4;
  position = [0 0 0]';

  orientation_rx = so3(Rx(phiz));
  qx = [quat(orientation_rx)'; position];

  orientation_ryrx = so3(Ry(phiz) * Rx(phiz));
  qyqx = [quat(orientation_ryrx)'; position];

  orientation_rzryrx = so3(Rz(phiz)* Ry(phiz) * Rx(phiz));
  qzqyqx = [quat(orientation_rzryrx)'; position];
  % q = [quat(orientation)'; position];

  % note = [ ...
  %   'Example 6: Cube Successively rotated by 45 degree about x, y, z axis \n', ...
  %   'JointPosition: ', sprintf('%.2f ', q')];


% part_b_2
elseif whichExample == 7
  % Body frame, Successive rotations for z->y->x with 45 degree for each one
  Rx =@(a) [1 0 0;
            0 cos(a) -sin(a);
            0 sin(a) cos(a)];

  Ry =@(a) [cos(a) 0 sin(a);
            0 1 0;
            -sin(a) 0 cos(a)];

  Rz =@(a) [cos(a) -sin(a) 0;
            sin(a) cos(a) 0;
            0 0 1];

  Rsucc =@(a) Rz(a) * Ry(a) * Rx(a);

  phiz = pi/4;
  position = [0 0 0]';
  orientation_rz = so3(Rz(phiz));
  qz = [quat(orientation_rz)'; position];

  orientation_rzry = so3(Rz(phiz) * Ry(phiz));
  qzqy = [quat(orientation_rzry)'; position];

  orientation_rzryrx = so3(Rz(phiz)* Ry(phiz) * Rx(phiz));
  qzqyqx = [quat(orientation_rzryrx)'; position];
  % q = [quat(orientation)'; position];
  % 
  % note = [ ...
  %   'Example 7: Cube Successively rotated by 45 degree about z, y, x axis \n', ...
  %   'JointPosition: ', sprintf('%.2f ', q')];

% end of b.
% ----------------------------------------------------------------------------

end





%----------------------------------------------------------------------------
% part b plotting
% b.1 (To see the right plot, please change whichExample to 6)
if whichExample == 6
    figure;
    
    subplot(1, 4, 1)
    show(cube, q_init);
    title("Initial Orientation");
    hold on;
    plotTransforms(pos_init', orient_init, 'FrameSize', 0.3);
    axis_style();
    
    subplot(1, 4, 2);
    show(cube, qx);
    hold on;
    title("Rx(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_rx, 'FrameSize', 0.3, 'FrameColor', [0, 0, 0]);
    axis_style();
    
    subplot(1, 4, 3);
    show(cube, qyqx);
    hold on;
    title("Ry(45 degree)Rx(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_ryrx, 'FrameSize', 0.3, 'FrameColor', [0, 0, 0]);
    axis_style();
    
    subplot(1, 4, 4);
    show(cube, qzqyqx);
    hold on;
    title("Rz(45 degree)Ry(45 degree)Rx(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_rzryrx, 'FrameSize', 0.3,'FrameColor', [0, 0, 0]);
    axis_style();

    % saveas(gcf, "PartB_1.jpg");
end

% b.2 (To see the right plot, please change whichExample to 7)
if whichExample == 7
    figure;
    
    subplot(1, 4, 1)
    show(cube, q_init);
    title("Initial Orientation");
    hold on;
    plotTransforms(pos_init', orient_init, 'FrameSize', 0.3);
    axis_style();
    
    subplot(1, 4, 2);
    show(cube, qz);
    hold on;
    title("Rz(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_rz, 'FrameSize', 0.3, 'FrameColor', [0, 0, 0]);
    axis_style();
    
    subplot(1, 4, 3);
    show(cube, qzqy);
    hold on;
    title("Rz(45 degree)Ry(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_rzry, 'FrameSize', 0.3, 'FrameColor', [0, 0, 0]);
    axis_style();
    
    subplot(1, 4, 4);
    show(cube, qzqyqx);
    hold on;
    title("Rz(45 degree)Ry(45 degree)Rx(45 degree)");
    plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)
    plotTransforms(pos_init', orientation_rzryrx, 'FrameSize', 0.3,'FrameColor', [0, 0, 0]);
    axis_style();

    % saveas(gcf, "PartB_2.jpg");
end
%----------------------------------------------------------------------------



%----------------------------------------------------------------------------
% part c

if whichExample == 8
    v = VideoWriter('animation.avi', 'Motion JPEG AVI');
    open(v);
    
    numOfFrames = 120;
    p0 = [0 0.25 0.25]';
    Ry =@(a) [cos(a) 0 sin(a);
            0 1 0;
            -sin(a) 0 cos(a)];
    Rz =@(a) [cos(a) -sin(a) 0;
            sin(a) cos(a) 0;
            0 0 1];
    
    % angular velocity for body frame
    w1 = 0.05;
    
    % angular velocity for rotating around space frame
    w2 = 0.05;
    
    
    figure;
    for i = 1:numOfFrames
        gcf; clf;
    
        theta1 = w1 * i;
        theta2 = w2 * i;
        
        % rotate 45 about y_body then continus rotating about z_body
        R_body = Ry(pi/4) * Rz(theta1);
    
        % rotating around z_space
        p = Rz(theta2) * p0;
        
        % quat config for cube self-rotation
        q = [quat(so3(R_body))'; p];
    
        % quat config for cube rotating around z_space
        q_rot = quat(so3(R_body));
       
        show(cube, q);
        hold on;
        
        plotTransforms([0 0 0], so3(eye(3)), 'Framesize', 0.5);
        plotTransforms(p', q_rot, 'Framesize', 0.5);
        axis_style()
    
        frame = getframe(gcf);
        writeVideo(v, frame);
        pause(0.01);
    end
    close(v);
end
% end of part c
%----------------------------------------------------------------------------





%----------------------------------------------------------------------------
% code from the oringial cube_code.m
% only for whichExample = 1-5
% Please manually change the title/filename
figure;
  % Display cube at current configuration
  show(cube,q); hold all;

  title("45 rotation about x_s axis");

  % Visualize space frame
  plotTransforms([0 0 0], so3(eye(3)), 'FrameSize',0.5)

  % Visualize body frame
  plotTransforms( ...
    position', orientation, 'FrameSize',0.25, 'FrameColor',[0 0 0 0.5]);

  % Remove defaut axis, set view (function is defined below)
  axis_style()

  ah = annotation('textbox',[0.2 0.2 0.7 0.1], 'String', sprintf(note))
  ah.EdgeColor = 'none';
  ah.FontSize = 14;

  saveas(gcf, "45_xAxis.jpg");
% -----------------------------------------------------------------------------

function axis_style()
  ax = gca;
  axis off;
  xlim([-0.5,0.5])
  ylim([-0.5,0.5])
  zlim([-0.5,0.5])
  ax.CameraViewAngle = 8;
end
