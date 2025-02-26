rosmasterIP = '192.168.56.101';
rosinit(rosmasterIP, 11311);  % Connect to ROS master on the VM

% Create a publisher for the /tf topic with message type tf2_msgs/TFMessage
tfStaticPub = rospublisher('/tf_static', 'tf2_msgs/TFMessage');

% Subscribe to the '/gazebo/link_states' topic
linkStatesSub = rossubscriber('/gazebo/link_states', 'gazebo_msgs/LinkStates');

% Static frames in simulation environment
staticLinks = {'table1::table::link', 'table2::table::link', 'BlueBin::Blue Bin::link', 'GreenBin::Green Bin::link', 'wcase::base', 'scale1::link_scale'};

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

% Publish the World_To_Frame_Transformation to /tf_static
function retrieveLinkPose(msg, name, tfStaticPub)

    % Process link=name if it exists in the message
    idx = find(strcmp(msg.Name, name), 1);
    if ~isempty(idx)

        T_world_link = msg.Pose(idx);
        T_world_link_msg = createTransformFromWorldToFrame(name, T_world_link, tfStaticPub);
        send(tfStaticPub, T_world_link_msg);
        fprintf('Broadcasted new transform from "world" to %s.\n', name);

    end

end

pause(2);
msg = receive(linkStatesSub, 10);

for i = 1:length(staticLinks)

    retrieveLinkPose(msg, staticLinks{i}, tfStaticPub);

end

rosshutdown;