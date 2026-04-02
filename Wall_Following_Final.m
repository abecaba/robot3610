function wallFollowSimple()
    clc;
    clear all;

    %% CONNECT HERE IS MY EDIT
clc
clear all

% Create an instance of the nanobot class
nb = nanobot('COM3', 115200, 'serial');


    %% RUN WALL TASK
    doWallFollow(nb);

    %% STOP + DISCONNECT
    nb.setMotor(1,0);
    nb.setMotor(2,0);

    delete(nb);
    clear('nb');
    clear all;
end


function doWallFollow(nb)
    %% INIT
    % FRONT ultrasonic: TRIGF = D22, ECHOF = D20
    nb.initUltrasonic1('D22','D20');

    % SIDE ultrasonic: TRIGS = D18, ECHOS = D16
    nb.initUltrasonic2('D18','D16');

    nb.initReflectance();

    %% TUNING VALUES
    baseSpeed = 8;       % slow is easier
    turnSpeed = 7;       % for rotate-to-find-wall
    maxDuty = 12;

    % these are RAW ultrasonic values for now
    frontDetectVal = 850;   % tune this
    sideDetectVal  = 1400;  % tune this
    targetVal      = 1000;  % tune this

    kp = 0.01;              % start small if using raw values
    kd = 0.0;               % add later if needed
    blackThresh = 250;      % tune to your tape / lighting

    %% STEP 1: DRIVE FORWARD UNTIL FRONT SENSOR SEES CYLINDER
    approachCylinder(nb, baseSpeed, frontDetectVal);

    %% STEP 2: ROTATE UNTIL SIDE SENSOR PICKS UP CYLINDER
    rotateUntilSideSeesWall(nb, turnSpeed, sideDetectVal);

    %% STEP 3: FOLLOW AROUND CYLINDER
    followAroundCylinder(nb, baseSpeed, maxDuty, targetVal, kp, kd, blackThresh);
end


function val = readFront(nb)
    val = nb.ultrasonicRead1();
end


function val = readSide(nb)
    val = nb.ultrasonicRead2();
end


function approachCylinder(nb, speed, frontDetectVal)
    nb.setMotor(1,0);
    nb.setMotor(2,0);
    pause(0.2);

    while true
        frontVal = readFront(nb);

        fprintf('Front = %d\n', frontVal);

        if frontVal > 0 && frontVal <= frontDetectVal
            break;
        end

        nb.setMotor(1, speed);
        nb.setMotor(2, speed);
        pause(0.03);
    end

    nb.setMotor(1,0);
    nb.setMotor(2,0);
    pause(0.2);
end


function rotateUntilSideSeesWall(nb, turnSpeed, sideDetectVal)
    % rotate right in place until side ultrasonic sees the cylinder

    nb.setMotor(1,0);
    nb.setMotor(2,0);
    pause(0.1);

    % If this turns the wrong way, swap the signs
    nb.setMotor(1, turnSpeed);
    nb.setMotor(2, -turnSpeed);

    while true
        sideVal = readSide(nb);

        fprintf('Side during rotate = %d\n', sideVal);

        % valid reading and close enough to know the side sensor found it
        if sideVal > 0 && sideVal <= sideDetectVal
            break;
        end

        pause(0.03);
    end

    nb.setMotor(1,0);
    nb.setMotor(2,0);
    pause(0.2);
end


function followAroundCylinder(nb, baseSpeed, maxDuty, targetVal, kp, kd, blackThresh)
    prevError = 0;
    prevTime = 0;

    tic;

    while true
        % ---- time step ----
        dt = toc - prevTime;
        prevTime = toc;

        if dt <= 0
            dt = 0.01;
        end

        % ---- side reading ----
        sideVal = readSide(nb);

        % skip bad reads
        if sideVal <= 0
            pause(0.03);
            continue;
        end

        % ---- stop when black line/bar is found again ----
        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        if all(vals > blackThresh)
            break;
        end

        % ---- control ----
        error = sideVal - targetVal;
        derivative = (error - prevError) / dt;

        control = kp*error + kd*derivative;

        % steer by slowing one side and speeding the other
        leftDuty  = baseSpeed + control;
        rightDuty = baseSpeed - control;

        % clamp
        leftDuty  = min(max(leftDuty, 0), maxDuty);
        rightDuty = min(max(rightDuty, 0), maxDuty);

        nb.setMotor(1, leftDuty);
        nb.setMotor(2, rightDuty);

        fprintf('Side=%d  Error=%.2f  L=%.2f  R=%.2f\n', ...
            sideVal, error, leftDuty, rightDuty);

        prevError = error;
        pause(0.03);
    end

    nb.setMotor(1,0);
    nb.setMotor(2,0);
    pause(0.2);
end

%% Front test
clc;
clear all;

nb = nanobot('COM3', 115200, 'serial');

% FRONT
nb.initUltrasonic1('D22','D20');
while true
    fprintf('Front = %d\n', nb.ultrasonicRead1());
    pause(0.2);
end

%% Side test
clc;
clear all;

nb = nanobot('COM3', 115200, 'serial');
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    '], 115200, 'serial');

% SIDE
nb.initUltrasonic2('D18','D16');
while true
    fprintf('Side = %d\n', nb.ultrasonicRead2());
    pause(0.2);
end