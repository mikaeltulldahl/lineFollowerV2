function [pose, vel] = propagateState(pose, twist, dt)
vel = polar2cartesian(pose.heading,twist.speed);
pose.heading = wrapAngleRad(pose.heading + dt*twist.yawRate);
pose.pos = pose.pos + dt*vel;
end