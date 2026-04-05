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

% Use this to change example output
whichExample = 2;

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
end


figure;
  % Display cube at current configuration
  show(cube,q); hold all;

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

% -----------------------------------------------------------------------------
function axis_style()
  ax = gca;
  axis off;
  xlim([-0.5,0.5])
  ylim([-0.5,0.5])
  zlim([-0.5,0.5])
  ax.CameraViewAngle = 8;
end
