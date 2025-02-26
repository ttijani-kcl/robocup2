rosmasterIP = '192.168.56.101';
rosinit(rosmasterIP, 11311);  % Connect to ROS master on the VM

% Publisher to /tf topic
tfPub = rospublisher('/tf', 'tf2_msgs/TFMessage');

% Create a tf buffer (listener) object
tfBuffer = rostf;

pause(2);

parentFrame = 'base_link';

% Retrieve the transform from 'world' to parentFrame
world_to_parentFrame = getTransform(tfBuffer, 'world', parentFrame);

% Assume tfBuffer is already created and populated
availableFrames = string(tfBuffer.AvailableFrames);

% Define the prefix to filter on
prefix = "item::";  % Use a string (double quotes) for string arrays

% Filter frames that start with the given prefix
itemFrames = availableFrames(startsWith(availableFrames, prefix));

% Create a rate controller for 10 Hz publishing
rateObj = rateControl(10);

function T_world_frame = createHomogeneousTransformationMatrix(world_to_frame)

    % Convert world_to_parentFrame to a homogeneous transformation matrix
    transFrame = [ world_to_frame.Transform.Translation.X, ...
                    world_to_frame.Transform.Translation.Y, ...
                    world_to_frame.Transform.Translation.Z ];
    quatFrame = [ world_to_frame.Transform.Rotation.W, ...
                    world_to_frame.Transform.Rotation.X, ...
                    world_to_frame.Transform.Rotation.Y, ...
                    world_to_frame.Transform.Rotation.Z ];
    T_world_frame = trvec2tform(transFrame) * quat2tform(quatFrame);

end

function tfMsg = createTransformationMessage(T_ParentFrame_ChildFrame, tfPub, parentFrame, childFrame)

    % Extract translation and rotation (quaternion) from T_ParentFrame_ChildFrame
    trans_ParentFrame_ChildFrame = tform2trvec(T_ParentFrame_ChildFrame);
    quat_ParentFrame_ChildFrame  = tform2quat(T_ParentFrame_ChildFrame);

    % Build a TransformStamped message with the computed transform
    tfMsg = rosmessage(tfPub);
    tformStamped = rosmessage('geometry_msgs/TransformStamped');

    tformStamped.Header.FrameId = parentFrame;
    tformStamped.Header.Stamp   = rostime('now');
    tformStamped.ChildFrameId   = childFrame;

    tformStamped.Transform.Translation.X = trans_ParentFrame_ChildFrame(1);
    tformStamped.Transform.Translation.Y = trans_ParentFrame_ChildFrame(2);
    tformStamped.Transform.Translation.Z = trans_ParentFrame_ChildFrame(3);

    tformStamped.Transform.Rotation.W = quat_ParentFrame_ChildFrame(1);
    tformStamped.Transform.Rotation.X = quat_ParentFrame_ChildFrame(2);
    tformStamped.Transform.Rotation.Y = quat_ParentFrame_ChildFrame(3);
    tformStamped.Transform.Rotation.Z = quat_ParentFrame_ChildFrame(4);

    tfMsg.Transforms = tformStamped;

end

function publishParentToChildFrameTransform(tfBuffer, tfPub, parentFrame, childFrame, world_to_parentFrame)

    % Retrieve the transform from 'world' to childFrame
    world_to_childFrame = getTransform(tfBuffer, 'world', childFrame);

    % Convert world_to_parentFrame to a homogeneous transformation matrix
    T_world_ParentFrame = createHomogeneousTransformationMatrix(world_to_parentFrame);

    % Convert world_to_childFrame to a homogeneous transformation matrix
    T_world_ChildFrame = createHomogeneousTransformationMatrix(world_to_childFrame);

    % Compute the transform from parentFrame to childFrame
    % Invert the parent's transform to get from parentFrame to world:
    T_ParentFrame_world = inv(T_world_ParentFrame);
    % Multiply by the child's transform to get from parentFrame to childFrame:
    T_ParentFrame_ChildFrame = T_ParentFrame_world * T_world_ChildFrame;

    % Build a TransformStamped message with the computed transform
    tfMsg = createTransformationMessage(T_ParentFrame_ChildFrame, tfPub, parentFrame, childFrame);

    % Publish the transform message
    send(tfPub, tfMsg);

end

% Duration for publishing (in seconds)
publishDuration = 200;
startTime = tic;  % Start timer

while toc(startTime) < publishDuration

    for i = 1:length(itemFrames)
        childFrame = itemFrames(i);
    
        publishParentToChildFrameTransform(tfBuffer, tfPub, parentFrame, childFrame, world_to_parentFrame);
    
    end

    % Wait to maintain the desired rate (10 Hz)
    waitfor(rateObj);

end

% Shutdown ROS after finishing
rosshutdown;