rosmasterIP = '192.168.56.101';
rosinit(rosmasterIP, 11311);  % Connect to ROS master on the VM

% Create a publisher for the /tf topic with message type tf2_msgs/TFMessage
tfPub = rospublisher('/tf', 'tf2_msgs/TFMessage');

% Subscribe to the '/gazebo/model_states' topic
sub = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates', @(src, msg) modelStatesCallback(src, msg, tfPub));

% Set loop duration (in seconds)
duration = 300;  % Duration in seconds
tic;            % Start the timer

while toc < duration
    % Your code here (e.g., display something, process data, etc.)
    pause(0.1); % Short pause to prevent overloading the CPU
end

delete(sub);

% Create TransformedStamped message for World_To_Frame_Transformation 
function tfMsg = createTransformFromWorldToFrame(name, pose, tfPub)

    % Create a tf message (tf2_msgs/TFMessage) that will contain our transform(s)
    tfMsg = rosmessage(tfPub);

    % Create a TransformStamped message (geometry_msgs/TransformStamped)
    tformStamped = rosmessage('geometry_msgs/TransformStamped');
    
    % Set the header: parent frame is 'world', and timestamp with the current time
    tformStamped.Header.FrameId = 'world';  
    tformStamped.Header.Stamp = rostime('now');  
    
    % Specify the child frame (the new frame you want to add)
    tformStamped.ChildFrameId = name;
    
    tformStamped.Transform.Translation.X = pose.Position.X;
    tformStamped.Transform.Translation.Y = pose.Position.Y;
    tformStamped.Transform.Translation.Z = pose.Position.Z;
    
    tformStamped.Transform.Rotation.X = pose.Orientation.X;
    tformStamped.Transform.Rotation.Y = pose.Orientation.Y;
    tformStamped.Transform.Rotation.Z = pose.Orientation.Z;
    tformStamped.Transform.Rotation.W = pose.Orientation.W;
    
    % Add the TransformStamped message to the tf message array
    tfMsg.Transforms = tformStamped;

end

% Publish the World_To_Frame_Transformation to /tf
function retrieveLinkPose(msg, name, tfPub)

    % Process link=name if it exists in the message
    idx = find(strcmp(msg.Name, name), 1);
    if ~isempty(idx)

        T_world_link = msg.Pose(idx);
        T_world_link_msg = createTransformFromWorldToFrame(['item::', name], T_world_link, tfPub);
        send(tfPub, T_world_link_msg);
        fprintf('Broadcasted new transform from "world" to %s.\n', name);

    end

end

% Callback to publish updated transforms to /tf when the
% gazebo/model_states is received
function modelStatesCallback(~, msg, tfPub)

    for i = 12:length(msg.Name)

        name = msg.Name{i};
        retrieveLinkPose(msg, name, tfPub);

    end

end

rosshutdown;