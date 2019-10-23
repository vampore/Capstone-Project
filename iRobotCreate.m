% A class for handling the iRobot Create
classdef iRobotCreate < handle
    properties
        X = 0;
        Y = 0;
        serialPort;
        serialObject;
        lastDistance;
        lastAngle;
        yaw = 0;
    end
    methods
        %% Constructor of the class, which initialize the robot and the important sensor
        function self = iRobotCreate(serialPort)
            if 0 < nargin
                self.serialObject = RoombaInit(serialPort);
                self.lastDistance = DistanceSensorRoomba(self.serialObject);
                self.lastAngle = AngleSensorRoomba(self.serialObject);               
            end
        end
        %% Move to a specific coordinate, the origin is at the robot initial place, robot moves in square pattern
        % y
        % |
        % |
        % |_ _ _ _ _ x
        function MoveSquareXY(self,xPos,yPos)
            xDiff = xPos - self.X;
            yDiff = yPos - self.Y;
            self.X = xPos;
            self.Y = yPos;
            if xDiff>0 && yDiff>0
                travelDist(self.serialObject,0.1,yDiff);
                turnAngle(self.serialObject,0.1,-85);
                travelDist(self.serialObject,0.1,xDiff);
                turnAngle(self.serialObject,0.1,85);
            elseif xDiff>0 && yDiff<0
                travelDist(self.serialObject,0.1,yDiff);
                turnAngle(self.serialObject,0.1,-85);
                travelDist(self.serialObject,0.1,xDiff);
                turnAngle(self.serialObject,0.1,85);
            elseif xDiff<0 && yDiff>0
                travelDist(self.serialObject,0.1,yDiff);
                turnAngle(self.serialObject,0.1,85);
                travelDist(self.serialObject,0.1,-xDiff);
                turnAngle(self.serialObject,0.1,-85);
            elseif xDiff < 0 && yDiff < 0 
                travelDist(self.serialObject,0.1,yDiff);
                turnAngle(self.serialObject,0.1,85);
                travelDist(self.serialObject,0.1,-xDiff);
                turnAngle(self.serialObject,0.1,-85);
            elseif xDiff > 0 && yDiff == 0
                turnAngle(self.serialObject,0.1,-85);
                travelDist(self.serialObject,0.1,xDiff);
                turnAngle(self.serialObject,0.1,85);
            elseif xDiff < 0 && yDiff == 0
                turnAngle(self.serialObject,0.1,85);
                travelDist(self.serialObject,0.1,-xDiff);
                turnAngle(self.serialObject,0.1,-85);
            elseif xDiff == 0 
                travelDist(self.serialObject,0.1,yDiff);
            end 
                
        end
        %% Move the robot given a distance and a yaw angle 
        function MoveYawDist(self,yaw,distance)
            % The yaw is made of the heading of the robot and the direction
            % of movement, yaw is positive in CCW
            turnAngle(self.serialObject,0.05,yaw);
            self.lastAngle = rad2deg(AngleSensorRoomba(self.serialObject));
            self.yaw = self.yaw + self.lastAngle;
            if self.lastAngle ~= yaw
                turnAngle(self.serialObject,0.05,yaw-self.lastAngle);
            end
            travelDist(self.serialObject,0.1,distance);
            self.lastDistance = DistanceSensorRoomba(self.serialObject);
            if self.lastDistance ~= distance
               travelDist(self.serialObject,0.1,distance-self.lastDistance);
            end
            self.X = self.X - distance*sin(yaw);
            self.Y = self.Y + distance*cos(yaw);
            self.lastDistance = DistanceSensorRoomba(self.serialObject);
            self.lastAngle = rad2deg(AngleSensorRoomba(self.serialObject));
            self.yaw = self.yaw + self.lastAngle;
        end
        %% Move to the next coordinate in straight line
        function MoveStraightXY(self,xPos,yPos)
            xDiff = xPos - self.X;
            yDiff = yPos - self.Y;
            self.X = xPos;
            self.Y = yPos;            
            angle = atan2(yDiff,xDiff);
            angleDiff = angle - self.yaw;
            self.yaw = angle;
            turnAngle(self.serialObject,0.05,angleDiff);
            d = sqrt((xDiff^2)+(yDiff^2));
            travelDist(self.serialObject,0.1,d);
            self.lastDistance = DistanceSensorRoomba(self.serialObject);
            travelDist(self.serialObject,0.1,d-self.lastDistance);
        end
    end
end
            
    