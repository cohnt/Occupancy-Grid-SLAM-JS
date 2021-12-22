/////////////////
/// CONSTANTS ///
/////////////////

var canvasSize = [0.49, 0.7]; //Size of each canvas, as a fraction of the total page width and height, respectively
var worldScale = 30; //The width of the entire browser viewport (in meters), determining the scale of the displays
var canvasLineWidth = 0.015;
var robotRadius = 0.2;
var robotMarkerTriangleAngle = 30 * (Math.PI / 180); //The front angle of the triangular robot marker
var robotStrokeStyle = "black";
var obstacleStrokeStyle = "black";
var lidarStrokeStyle = "red";
var obstacleSizeRange = [0.5, 2.5];
var numObstacles = 50;
var robotSpeed = 1.0; // Robot speed, in meters per second
var robotTurnRate = 120 * (Math.PI / 180); // Robot turn rate, in radians per second
var lidarNumPoints = 60; // Number of points given in each sweep of the lidar
var lidarFOV = 360 * (Math.PI / 180); // FOV of the lidar, in radians
var lidarAngle = lidarFOV / (lidarNumPoints - 1); // The angle between two lidar beams
var lidarNoiseVariance = 0.025; //The variance of the noise affecting the lidar measurements, in meters.
var cellWidth = 0.1; //The width of each occupancy grid cell, in meters
var occupancyTrust = 4;
var maxLog = 5;
var minLog = -5;
var distMax = 30;

var numParticles = 250; //Number of samples to use for the particle filter.
var particlePosNoiseVariance = 0.01; //The variance of the diffusion noise added to the position during resampling.
var particleOrientationNoiseVariance = 2.5 * (Math.PI / 180); //The variance of the diffusion noise added to the orientation during resampling.
var explorationFactor = 0; //0.0 means no particles are randomly placed for exploration, 0.5 means 50%, 1.0 means 100%
var useExplorationParticlesGuess = false; //Whether or not to use exploration particles when estimating robot pose.
var useBestParticle = false; //If true, just select the best particle as the ground truth, instead of averaging.

var odomPosNoiseVar = 0.005;
var odomOrienNoiseVar = 1 * (Math.PI / 180)

var particleDispRadius = 0.025; //Radius of the circle marker for each particle.
var particleDispHeadingLength = 0.05; //Length of the direction marker for each particle.
var errorWeightColorDivisor = 300; //Used when selecting the color to correspond with each particle.
var weightColorMultiplier = 0.9; //Used when selecting the color to correspond with each particle.

////////////////////////
/// GLOBAL VARIABLES ///
////////////////////////

var worldCanvas; //HTML element object of the world canvas
var mapCanvas; //HTML element object of the map canvas
var worldCtx; //Canvas drawing context of the world canvas
var mapCtx; //Canvas drawing context of the map context
var keyStates = {}; //Status of each (keyboard) key

var hasStarted = false; //Used for the control of the tick loop.
var running = false; //Used for the control of the tick loop.
var stop = false; //Used for the control of the tick loop.
var moved = false; //Used for the control of the tick loop.

var pixelsPerMeter; //Pixels per meter
var worldWidth; //World width in meters
var worldHeight; //World height in meters
var worldMaxX; //The maximum x coordinate shown in the world, i.e., worldWidth/2
var worldMaxY; //The maximum y coordinate shown in the world, i.e., worldHeight/2

var obstacles = []; //A list of obstacle objects
var obstacleSegments = []; //A list of all segments of all obstacles
var robotPose;
var robotPoseHistory = [];
var lastFrameTime;
var lidarDistances = []; //When simulating what the LIDAR sensor would see, this contains all of the distance readings.
var lidarEnds = []; //The endpoints of each LIDAR beam.

var gridWidth;
var gridHeight;
var occupancyGrid = [];

var estRobotPose;
var particles = [];
var maxWeight = 0;

///////////////
/// CLASSES ///
///////////////

function Pose(pos, orien) {
	this.pos = pos.slice();
	this.orien = orien; //Given in raidans
}
function Obstacle(pos, orien, width) {
	this.pos = pos.slice();
	this.orien = orien; //Given in raidans
	this.width = width;

	this.draw = function(ctx) {
		var segments = this.segments();

		ctx.strokeStyle = obstacleStrokeStyle;
		for(var i=0; i<segments.length; ++i) {
			ctx.beginPath();
			ctx.moveTo(segments[i][0][0], segments[i][0][1]);
			ctx.lineTo(segments[i][1][0], segments[i][1][1]);
			ctx.stroke();
		}
	}

	this.corners = function() {
		var dx = 0.5 * this.width * Math.cos(this.orien);
		var dy = 0.5 * this.width * Math.sin(this.orien);

		var corners = [
			[this.pos[0]+dx, this.pos[1]+dy],
			[this.pos[0]+dy, this.pos[1]-dx],
			[this.pos[0]-dx, this.pos[1]-dy],
			[this.pos[0]-dy, this.pos[1]+dx]
		];

		return corners;
	}

	this.segments = function() {
		var corners = this.corners();

		var segments = [
			[corners[0], corners[1]],
			[corners[1], corners[2]],
			[corners[2], corners[3]],
			[corners[3], corners[0]]
		];

		return segments;
	}
}
function randomObstacle() {
	var width = (Math.random() * (obstacleSizeRange[1] - obstacleSizeRange[0])) + obstacleSizeRange[0];
	var pos = [null, null];
	do {
		pos[0] = Math.random() * worldWidth - worldMaxX;
		pos[1] = Math.random() * worldHeight - worldMaxY;
	}
	while(
		pos[0] < robotRadius + width
		&&
		pos[0] > robotRadius - width
		&&
		pos[1] < robotRadius + width
		&&
		pos[1] > robotRadius - width
	);
	var orien = Math.random() * 2 * Math.PI;
	return new Obstacle(pos, orien, width);
}
//Each particle is a guess at where the robot may be, coupled with information about what that guess entails.
function Particle(pos=[0,0], orien=0) {
	this.pos = pos.slice(); //The guessed position of the robot.
	this.orien = orien; //The guessed orientation of the robot.
	this.weight = 0; //The weight of the particle reflects how well the guess matches the sensor observations.
	this.lidarReadings = new Array(lidarNumPoints).fill(0); //The LIDAR sensor readings, simulated as if the robot were precisely at this.pos.
	this.isExploration = false; //Exploration particles can be flagged, so that they aren't used when averaging particles to compute the predicted robot location.

	this.randomize = function() {
		//This simply moves the particle to a random pose in the world.
		this.pos = [
			(Math.random() * worldWidth) - worldMaxX,
			(Math.random() * worldHeight) - worldMaxY
		];
		this.orien = Math.random() * 2 * Math.PI - Math.PI; //I.e. uniformly distributed between -Pi and Pi.
		this.isExploration = true;
	}

	this.isValid = function() {
		// return Math.abs(this.pos[0]) < worldMaxX && Math.abs(this.pos[1]) < worldMaxY;
		return true; //This was unhelpful
	}

	this.draw = function(ctx, maxWeight) {
		//We need to know maxWeight for weightToColor.
		color = weightToColor(this.weight / maxWeight);
		ctx.strokeStyle = color;
		ctx.fillStyle = color;
		ctx.beginPath();
		ctx.moveTo(this.pos[0], this.pos[1]);
		ctx.arc(this.pos[0], this.pos[1], particleDispRadius, 0, 2*Math.PI, true);
		ctx.closePath();
		ctx.fill();
		ctx.beginPath();
		ctx.moveTo(this.pos[0], this.pos[1]);
		ctx.lineTo(this.pos[0] + (particleDispHeadingLength*Math.cos(this.orien)),
			this.pos[1] + (particleDispHeadingLength*Math.sin(this.orien)));
		ctx.stroke();
	}
}

/////////////////
/// FUNCTIONS ///
/////////////////

function setup() {
	worldCanvas = document.getElementById("worldCanvas");
	mapCanvas = document.getElementById("mapCanvas");

	worldCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	worldCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");
	mapCanvas.setAttribute("width", String(window.innerWidth * canvasSize[0]) + "px");
	mapCanvas.setAttribute("height", String(window.innerHeight * canvasSize[1]) + "px");

	// Create event listeners for the various controls on the page
	document.getElementById("startButton").addEventListener("click", startButtonClick);
	document.getElementById("pauseButton").addEventListener("click", pauseButtonClick);
	document.getElementById("resetButton").addEventListener("click", resetButtonClick);
	document.getElementById("newWorldButton").addEventListener("click", newWorldButtonClick);
	document.addEventListener("keydown", function(e) {
		keydownHandler(e);
	});
	document.addEventListener("keyup", function(e) {
		keyupHandler(e);
	});

	//World parameters
	pixelsPerMeter = window.innerWidth / worldScale;
	worldWidth = canvasSize[0] * window.innerWidth / pixelsPerMeter;
	worldHeight = canvasSize[1] * window.innerHeight / pixelsPerMeter;
	worldMaxX = worldWidth / 2;
	worldMaxY = worldHeight / 2;

	//Occupancy grid parameters
	gridWidth = Math.ceil(worldWidth / cellWidth);
	gridHeight = Math.ceil(worldHeight / cellWidth);
	if(gridWidth % 2 == 0) {
		++gridWidth;
	}
	if(gridHeight % 2 == 0) {
		++gridHeight;
	}

	//Create the canvas contexts
	worldCtx = worldCanvas.getContext("2d");
	mapCtx = mapCanvas.getContext("2d");

	resetCtx(worldCtx);
	resetCtx(mapCtx);

	generateWorld();
	reset();
}

function startButtonClick() {
	//This is the callback function if you click the start button.
	if(!running && !hasStarted) {
		//If we aren't already running, and we haven't started yet, start for the first time.
		reset();
		running = true;
		hasStarted = true;
		lastFrameTime = null;
		tick(); //This is the actual loop function. You only need to call it once -- it will keep calling itself as appropriate.
	}
	else if(!running && hasStarted) {
		//If we aren't running, but we have started yet, resume where we left off.
		running = true;
		lastFrameTime = null;
		tick(); //This is the actual loop function. You only need to call it once -- it will keep calling itself as appropriate.
	}
	//If we are running, do nothing.
}
function pauseButtonClick() {
	//This is the callback function if you click the pause button.
	if(running) {
		//If you click the pause button while we're running, stop. Otherwise, do nothing.
		stop = true;
	}
	//Note that the actual process of stopping is handled within the tick() function.
	//This just lets the function know to stop the next time around.
}
function resetButtonClick() {
	//This is the callback function if you click the reset button.
	if(!running) {
		hasStarted = false;
		moved = false;
		reset(); //Get everything back to its initial state.
	}
}
function newWorldButtonClick() {
	//This is the callback function if you click the new world button.
	if(!running) {
		//If we aren't currently running, generate a new world, and reset.
		//resetContext() isn't needed, because the canvas itself isn't changing.
		hasStarted = false;
		moved = false;
		clearWorld();
		generateWorld();
		reset();
	}
}
function keydownHandler(e) {
	var keyId = e.which;
	keyStates[keyId] = true;
}
function keyupHandler(e) {
	var keyId = e.which;
	keyStates[keyId] = false;
}

function resetCtx(ctx) {
	ctx.setTransform(1, 0, 0, 1, 0, 0); //Reset the transformation
	ctx.transform(1, 0, 0, -1, 0, 0); //Make y+ point up
	ctx.transform(1, 0, 0, 1, ctx.canvas.width/2, -ctx.canvas.height/2); //Center 0,0 in the middle of the canvas
	ctx.transform(pixelsPerMeter, 0, 0, pixelsPerMeter, 0, 0); //Scale according browser scaling
	ctx.lineWidth = canvasLineWidth; //Set the appropriate line width
}
function clearCanvas(ctx) {
	var tf = ctx.getTransform(); //Get the current transformation
	ctx.setTransform(1, 0, 0, 1, 0, 0); //Reset the transformation
	ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height); //Clear the canvas
	ctx.setTransform(tf); //Restore the previous transformation
}
function drawRobot(ctx, pose) {
	//pose should be a Pose object
	//Draw the outer circle
	ctx.strokeStyle = robotStrokeStyle;
	ctx.beginPath();
	//We initially move to a position on the circle itself, so that there's no weird line from the center to the circle.
	//Just JavaScript things! :)
	ctx.moveTo(pose.pos[0] + robotRadius, pose.pos[1]);
	ctx.arc(pose.pos[0], pose.pos[1], robotRadius, 0, 2*Math.PI, true);
	ctx.stroke();

	//Draw a triangle showing orientation.
	//First, compute the coordinates of the three points.
	var dx = robotRadius * Math.cos(pose.orien)
	var dy = robotRadius * Math.sin(pose.orien)
	var front = [pose.pos[0] + dx, pose.pos[1] + dy]

	var backLeftAngle = pose.orien + Math.PI - robotMarkerTriangleAngle;
	dx = robotRadius * Math.cos(backLeftAngle);
	dy = robotRadius * Math.sin(backLeftAngle);
	var backLeft = [pose.pos[0] + dx, pose.pos[1] + dy];

	var backRightAngle = pose.orien + Math.PI + robotMarkerTriangleAngle;
	dx = robotRadius * Math.cos(backRightAngle);
	dy = robotRadius * Math.sin(backRightAngle);
	var backRight = [pose.pos[0] + dx, pose.pos[1] + dy];
	
	//Now actually draw the triangle.
	ctx.beginPath();
	ctx.moveTo(front[0], front[1]);
	ctx.lineTo(backLeft[0], backLeft[1]);
	ctx.lineTo(backRight[0], backRight[1]);
	ctx.lineTo(front[0], front[1]);
	ctx.closePath();
	ctx.stroke();
	ctx.fillStyle = "black";
	ctx.fill();
}
function drawLidar(ctx) {
	ctx.strokeStyle = lidarStrokeStyle;
	ctx.beginPath();
	for(var i=0; i<lidarEnds.length; ++i) {
		ctx.moveTo(robotPose.pos[0], robotPose.pos[1]);
		ctx.lineTo(lidarEnds[i][0], lidarEnds[i][1]);
	}
	ctx.stroke();
}
function drawFrame() {
	clearCanvas(worldCtx);
	clearCanvas(mapCtx);

	//Draw the obstacles onto the world
	for(var i=0; i<obstacles.length; ++i) {
		obstacles[i].draw(worldCtx);
	}
	//Draw the robot onto the world
	drawRobot(worldCtx, robotPose);
	drawLidar(worldCtx);
	drawGrid(mapCtx);
	drawRobot(mapCtx, estRobotPose);

	if(moved) {
		drawParticles();
	}
}

function reset() {
	lidarDistances = [];
	lidarEnds = [];
	robotPose = new Pose([0, 0], 0);
	estRobotPose = robotPose;
	robotPoseHistory = [];
	constructGrid();
	resetParticleFilter();
	drawFrame();
}
function tick() {
	//This is the function where it all happens. It will repeatedly call itself until stopped.
	if(stop) {
		//Stop is set to true if the user pauses.
		running = false;
		stop = false;
		return; //We return early, so all the code isn't executed.
	}

	if(!lastFrameTime) {
		lastFrameTime = getTimeMS();
	}
	var dt = getTimeMS() - lastFrameTime;
	lastFrameTime += dt;

	updateRobotPos(dt);
	lidarDistances = computeLidarDists(robotPose);
	lidarEnds = [];
	noisifyLidar();
	computeLidarEndpoints(robotPose);

	if(moved) {
		var dPos = [
			robotPose.pos[0] - robotPoseHistory[robotPoseHistory.length-1].pos[0],
			robotPose.pos[1] - robotPoseHistory[robotPoseHistory.length-1].pos[1]
		];
		var dOrien = robotPose.orien - robotPoseHistory[robotPoseHistory.length-1].orien;
		propogateParticles(dPos, dOrien);
		measureParticles(); //This computes the simulated LIDAR observation for every particle.
		calculateWeights(); //This computes the weights for each particle, based on that observation.
		estRobotPose = makePrediction(); //This predicts the location of the robot, based on the particles and their weights.
	}
	else {
		estRobotPose = robotPose;
		//If the robot hasn't moved, use the real robot pose as the particle filter estimate.
		//It will always just be (0,0).
	}
	updateOccupancyGrid(estRobotPose);

	drawFrame();

	if(moved) {
		resample(); //This is the function where new particles are created based on the old particles.
	}

	robotPoseHistory.push(JSON.parse(JSON.stringify(robotPose)));

	requestAnimationFrame(tick);
}
function getTimeMS() {
	//
	return (new Date()).getTime();
}

function clearWorld() {
	obstacles = [];
	obstacleSegments = [];
}
function generateWorld() {
	for(var i=0; i<numObstacles; ++i) {
		obstacles.push(randomObstacle());
	}
	for(var i=0; i<numObstacles; ++i) {
		var segments = obstacles[i].segments();
		for(var j=0; j<segments.length; ++j) {
			obstacleSegments.push(segments[j]);
		}
	}

	//Add in the outer walls
	obstacleSegments.push([
		[worldMaxX, worldMaxY],
		[worldMaxX, -1*worldMaxY]
	]);
	obstacleSegments.push([
		[worldMaxX, -1*worldMaxY],
		[-1*worldMaxX, -1*worldMaxY]
	]);
	obstacleSegments.push([
		[-1*worldMaxX, -1*worldMaxY],
		[-1*worldMaxX, worldMaxY]
	]);
	obstacleSegments.push([
		[-1*worldMaxX, worldMaxY],
		[worldMaxX, worldMaxY]
	]);
}

function updateRobotPos(dt) {
	//This function is run every tick loop, based on the keys that are being held down at that time.
	var upKey = 87; //W
	var leftKey = 65; //A
	var downKey = 83; //S
	var rightKey = 68; //D

	var orienChange = 0;
	var posChange = [0, 0];

	var ds = dt / 1000; //Change in time, in seconds

	//Key are undefined before they're first pressed, but applying the ! operator twice converts undefined to false.
	//I.e., !undefined == true, !!undefined == false

	//First, handle orientation change.
	if((!!keyStates[leftKey]) && !keyStates[rightKey]) {
		//If we're trying to turn left and not right...
		orienChange = ds * robotTurnRate;
		robotPose.orien += orienChange;
	}
	else if(!keyStates[leftKey] && (!!keyStates[rightKey])) {
		//If we're trying to turn right and not left...
		orienChange = ds * robotTurnRate * -1;
		robotPose.orien += orienChange;
	}

	//Now, handle position change.
	if((!!keyStates[upKey]) && !keyStates[downKey]) {
		//If we're trying to go forward and not backward...
		var dx = ds * robotSpeed * Math.cos(robotPose.orien);
		var dy = ds * robotSpeed * Math.sin(robotPose.orien);
		var newPos = [
			robotPose.pos[0] + dx,
			robotPose.pos[1] + dy
		];
		if(!isColliding(newPos)) {
			//If we're not driving into a wall or off the map, update the position.
			posChange = [dx, dy];
		}
	}
	else if(!keyStates[upKey] && (!!keyStates[downKey])) {
		//If we're trying to go backward and not forward...
		var dx = ds * robotSpeed * Math.cos(robotPose.orien) * -1;
		var dy = ds * robotSpeed * Math.sin(robotPose.orien) * -1;
		var newPos = [
			robotPose.pos[0] + dx,
			robotPose.pos[1] + dy
		];
		if(!isColliding(newPos)) {
			//If we're not driving into a wall or off the map, update the position.
			posChange = [dx, dy];
		}
	}
	robotPose.pos[0] += posChange[0];
	robotPose.pos[1] += posChange[1];
	if(!moved) {
		if(posChange[0] != 0 || posChange[1] != 0 || orienChange != 0) {
			moved = true;
		}
	}
}
function isColliding(pos) {
	for(var i=0; i<obstacleSegments.length; ++i) {
		if(lineCircleCollisionTest(obstacleSegments[i], pos, robotRadius)) {
			return true;
		}
	}
	return false;
}
function computeLidarDists(pose) {
	var dists = [];
	for(var lidarIdx=0; lidarIdx<lidarNumPoints; ++lidarIdx) {
		var robotFrameAngle = (-lidarFOV / 2) + (lidarIdx * lidarAngle);
		var globalFrameAngle = robotFrameAngle + pose.orien;
		var dx = Math.cos(globalFrameAngle) * (worldWidth + worldHeight) * 43;
		var dy = Math.sin(globalFrameAngle) * (worldWidth + worldHeight) * 43;
		var lidarBeam = [
			pose.pos.slice(),
			[
				pose.pos[0] + dx,
				pose.pos[1] + dy
			]
		];

		var found = false;
		var bestDist = distMax;
		var bestIntersection = lidarBeam[1];
		for(var i=0; i<obstacleSegments.length; ++i) {
			var intersection = lineLineIntersection(obstacleSegments[i], lidarBeam);
			if(intersection) {
				found = true;
				var dist = distance(pose.pos, intersection);
				if(dist < bestDist) {
					bestDist = dist;
				}
			}
		}
		dists.push(bestDist);
	}
	return dists;
}
function noisifyLidar() {
	//This adds normally-distributed noise to a list of LIDAR readings.
	for(var i=0; i<lidarDistances.length; ++i) {
		lidarDistances[i] += randomNormal(0, lidarNoiseVariance);
		if(lidarDistances[i] < 0) {
			lidarDistances[i] = 0;
		}
	}
}
function computeLidarEndpoints(pose) {
	for(var i=0; i<lidarDistances.length; ++i) {
		var robotFrameAngle = (-lidarFOV / 2) + (i * lidarAngle);
		var globalFrameAngle = robotFrameAngle + pose.orien;
		var dist = lidarDistances[i] == Infinity ? 43*(worldWidth + worldHeight) : lidarDistances[i];
		lidarEnds.push([
			pose.pos[0] + (dist * Math.cos(globalFrameAngle)),
			pose.pos[1] + (dist * Math.sin(globalFrameAngle))
		]);
	}
}

function distance(p1, p2) {
	var total = 0;
	for(var i=0; i<p1.length; ++i) {
		total += Math.pow(p1[i]-p2[i], 2)
	}
	return Math.sqrt(total);
}
function lineCircleCollisionTest(line, circleCenter, radius) {
	// https://stackoverflow.com/a/37225895/
	var v1, v2, v3, u;
	v1 = [0, 0];
	v2 = [0, 0];
	v3 = [0, 0];
	v1[0] = line[1][0] - line[0][0];
	v1[1] = line[1][1] - line[0][1];
	v2[0] = circleCenter[0] - line[0][0];
	v2[1] = circleCenter[1] - line[0][1];
	u = (v2[0] * v1[0] + v2[1] * v1[1]) / (v1[1] * v1[1] + v1[0] * v1[0]); // unit dist of point on line
	if(u >= 0 && u <= 1){
		v3[0] = (v1[0] * u + line[0][0]) - circleCenter[0];
		v3[1] = (v1[1] * u + line[0][1]) - circleCenter[1];
		v3[0] *= v3[0];
		v3[1] *= v3[1];
		return Math.sqrt(v3[1] + v3[0]) < radius; // return distance from line
	} 
	// get distance from end points
	v3[0] = circleCenter[0] - line[1][0];
	v3[1] = circleCenter[1] - line[1][1];
	v3[0] *= v3[0];  // square vectors
	v3[1] *= v3[1];
	v2[0] *= v2[0];
	v2[1] *= v2[1];
	return Math.min(Math.sqrt(v2[1] + v2[0]), Math.sqrt(v3[1] + v3[0])) < radius; // return smaller of two distances as the result
}
function lineLineIntersection(l1, l2) {
	// https://stackoverflow.com/a/54866504/
	var pointA = l1[0];
	var pointB = l1[1];
	var pointC = l2[0];
	var pointD = l2[1];
	var z1 = (pointA[0] - pointB[0]);
	var z2 = (pointC[0] - pointD[0]);
	var z3 = (pointA[1] - pointB[1]);
	var z4 = (pointC[1] - pointD[1]);
	var dist = z1 * z4 - z3 * z2;
	if (dist == 0) {
		return false;
	}
	var tempA = (pointA[0] * pointB[1] - pointA[1] * pointB[0]);
	var tempB = (pointC[0] * pointD[1] - pointC[1] * pointD[0]);
	var xCoor = (tempA * z2 - z1 * tempB) / dist;
	var yCoor = (tempA * z4 - z3 * tempB) / dist;

	if (xCoor < Math.min(pointA[0], pointB[0]) || xCoor > Math.max(pointA[0], pointB[0]) ||
		xCoor < Math.min(pointC[0], pointD[0]) || xCoor > Math.max(pointC[0], pointD[0])) {
		return false;
	}
	if (yCoor < Math.min(pointA[1], pointB[1]) || yCoor > Math.max(pointA[1], pointB[1]) ||
		yCoor < Math.min(pointC[1], pointD[1]) || yCoor > Math.max(pointC[1], pointD[1])) {
		return false;
	}

	return [xCoor, yCoor];
}

function drawGrid(ctx) {
	for(var i=0; i<occupancyGrid.length; ++i) {
		for(var j=0; j<occupancyGrid[i].length; ++j) {
			var notLogProb = getProbFromLog(occupancyGrid[i][j]);
			var intColor = Math.floor(notLogProb * 256);
			if(intColor == 256) {
				intColor = 255;
			}
			var hexColor = intColor.toString(16);
			var colorCode = "#" + String(hexColor) + String(hexColor) + String(hexColor);
			ctx.fillStyle = colorCode;

			var pos = gridIdxToXY(i, j);
			ctx.fillRect(pos[0] - cellWidth, pos[1] - cellWidth, cellWidth, cellWidth);
			ctx.fill();
		}
	}
}
function constructGrid() {
	occupancyGrid = [];
	for(var i=0; i<gridHeight; ++i) {
		occupancyGrid.push([]);
		for(var j=0; j<gridWidth; ++j) {
			occupancyGrid[i].push(0.5);
		}
	}
}
function gridIdxToXY(i, j) {
	var x = (j - ((gridWidth-1)/2)) * cellWidth;
	var y = (((gridHeight-1)/2) - i) * cellWidth;
	return [x, y];
}
function xyToGridIdx(pos) {
	var j = Math.round(pos[0] / cellWidth) + ((gridWidth-1)/2);
	var i = ((gridHeight-1)/2) - Math.round(pos[1] / cellWidth);
	return [i, j];
}
function bresenham(lidarBeam) {
	//Returns a list of grid cells passed over by the lidar beam, in the form of index pairs [row, col]
	//Stops when it reaches the endpoint, or when it leaves the map
	var p1 = xyToGridIdx(lidarBeam[0]);
	var p2 = xyToGridIdx(lidarBeam[1]);
	//https://stackoverflow.com/a/4672319
	var passedCoords = [];

	if(isNaN(p1[0]) || isNaN(p1[1]) || isNaN(p2[0]) || isNaN(p2[1])) {
		return passedCoords;
	}

	var dx = Math.abs(p2[0]-p1[0]);
	var dy = Math.abs(p2[1]-p1[1]);
	var sx = (p1[0] < p2[0]) ? 1 : -1;
	var sy = (p1[1] < p2[1]) ? 1 : -1;
	var err = dx - dy;

	var i = p1[0];
	var j = p1[1];

	loopCount = 0
	while(true) {
		++loopCount;
		if(loopCount > 1000) {
			alert("UH OH")
		}
		if(i >= gridHeight || i < 0 || j >= gridWidth || j < 0) {
			break;
		}
		passedCoords.push([i, j]);
		if(i == p2[0] && j == p2[1]) {
			break;
		}
		var err2 = 2*err;
		if(err2 > -dy) {
			err -= dy;
			i += sx;
		}
		if(err2 < dx) {
			err += dx;
			j += sy;
		}
	}

	return passedCoords;
}
function updateOccupancyGrid(pose) {
	var lidarBeams = [];
	for(var i=0; i<lidarDistances.length; ++i) {
		lidarBeams.push([]);
		lidarBeams[i].push(pose.pos.slice());
		var robotFrameAngle = (-lidarFOV / 2) + (i * lidarAngle);
		var globalFrameAngle = robotFrameAngle + pose.orien;
		var dist = lidarDistances[i] == Infinity ? 43*(worldWidth + worldHeight) : lidarDistances[i];
		lidarBeams[i].push([
			pose.pos[0] + (dist * Math.cos(globalFrameAngle)),
			pose.pos[1] + (dist * Math.sin(globalFrameAngle))
		]);
	}
	for(var i=0; i<lidarBeams.length; ++i) {
		var passedCoords = bresenham(lidarBeams[i]);
		if(passedCoords.length == 0) {
			continue
		}
		
		var lastIdx = passedCoords.length - 1;
		var lastCoord = passedCoords[lastIdx];
		var endPoint = xyToGridIdx(lidarBeams[i][1]);
		passedCoords.pop();
		
		//https://natanaso.github.io/ece276a2019/ref/ECE276A_9_PF_SLAM.pdf

		//Update last coordinate to increase probability of occupancy
		occupancyGrid[lastCoord[0]][lastCoord[1]] += Math.log(occupancyTrust);

		//Update rest of the coordinates to decrease probability of occupancy
		for(var j=0; j<passedCoords.length; ++j) {
			occupancyGrid[passedCoords[j][0]][passedCoords[j][1]] -= Math.log(occupancyTrust);
		}
	}
	for(var i=0; i<occupancyGrid.length; ++i) {
		for(var j=0; j<occupancyGrid[i].length; ++j) {
			occupancyGrid[i][j] = Math.min(Math.max(occupancyGrid[i][j], minLog), maxLog);
		}
	}
}

function drawParticles() {
	for(var i=0; i<particles.length; ++i) {
		particles[i].draw(mapCtx, maxWeight)
	}
}
function resetParticleFilter() {
	particles = [];
	for(var i=0; i<numParticles; ++i) {
		particles.push(new Particle());
	}
	noisifyParticles();
}
function measureParticles() {
	//This function computes the LIDAR readings for every particle.
	//It also moves particles that would be off the world or in a wall to a random location.
	for(var i=0; i<numParticles; ++i) {
		//For each particle...
		if(!particles[i].isValid()) {
			//If it's off the world or in a wall, randomize it.
			particles[i].randomize();
		}
		//Compute the LIDAR readings for the particle. We use the same function as that for computing the robot's LIDAR readings.
		particles[i].lidarReadings = computeLidarDists(particles[i]); //TODO: Make this use the already built map
	}
}
function calculateWeights() {
	//This function computes the weights for every particle.
	var lidarDataArr = []; //This array will contain the difference between the observed LIDAR reading and the particle's simulated reading.
	var lidarDataWeights = []; //This array will contain the weights.
	for(var i=0; i<lidarNumPoints; ++i) {
		lidarDataArr[i] = particles.map(a => Math.abs(a.lidarReadings[i] - lidarDistances[i])); //Get the differences in distance.
		lidarDataWeights[i] = normalizeWeight(weightFromDistance(lidarDataArr[i])); //Convert those differences into a weight.
	}
	//Combine and normalize the weights.
	var combinedWeights = [];
	for(var i=0; i<numParticles; ++i) {
		combinedWeights[i] = 1;
		for(var j=0; j<lidarNumPoints; ++j) {
			combinedWeights[i] *= lidarDataWeights[j][i];
		}
	}

	maxWeight = 0;
	combinedWeights = normalizeWeight(combinedWeights); //Normalize again.
	for(var i=0; i<particles.length; ++i) {
		particles[i].weight = combinedWeights[i]; //Update the particle weights.
		maxWeight = Math.max(maxWeight, particles[i].weight);
	}
}
function makePrediction() {
	//Given all of the weighted particles, predict the robot's location via a weighted average.
	//One could also use the MLE or MAP particles.
	if(useBestParticle) {
		bestWeight = particles[0].weight;
		bestWeightIdx = 0;
		for(var i=1; i<particles.length; ++i) {
			if(particles[i].weight > bestWeight) {
				bestWeight = particles[i].weight;
				bestWeightIdx = i;
			}
		}
		return new Pose(particles[bestWeightIdx].pos, particles[bestWeightIdx].orien);
	}
	else {
		var totalPos = [0, 0];
		var totalOrien = 0;
		var totalWeight = 0;
		for(var i=0; i<particles.length; ++i) {
			if(!particles[i].isExploration || useExplorationParticlesGuess) {
				//Check if a particle is exploration before using it in our estimate.
				totalPos[0] += particles[i].pos[0] * particles[i].weight;
				totalPos[1] += particles[i].pos[1] * particles[i].weight;
				totalOrien += particles[i].orien * particles[i].weight;
				totalWeight += particles[i].weight;
			}
		}
		totalPos[0] /= totalWeight;
		totalPos[1] /= totalWeight;
		totalOrien /= totalWeight;
		return new Pose(totalPos, totalOrien);
	}
}
function sortParticles(particles) {
	//This sorts the particles by weight. I got this code from Stack Overflow.
	sorted = particles.slice()
	sorted.sort((a, b) => (a.weight > b.weight) ? 1 : -1);
	return sorted;
}
function resample() {
	//Resampling step. This implements importance sampling.
	//It's implemented slightly strangely, because Javascript doesn't have a great math library.
	var weightData = particles.map(a => a.weight);
	var cs = cumsum(weightData);
	var step = 1/((numParticles * (1 - explorationFactor))+1);
	var chkVal = step;
	var chkIndex = 0;
	for(var i=0; i<numParticles * (1 - explorationFactor); ++i) {
		while(cs[chkIndex] < chkVal) {
			++chkIndex;
		}
		chkVal += step;
		var newPos = particles[chkIndex].pos;
		var newOrien = particles[chkIndex].orien;
		newPos[0] += randomNormal(0, particlePosNoiseVariance);
		newPos[1] += randomNormal(0, particlePosNoiseVariance);
		newOrien += randomNormal(0, particleOrientationNoiseVariance);
		particles[i] = new Particle(newPos, newOrien);
	}
	for(var i=Math.floor(numParticles * (1 - explorationFactor)); i<numParticles; ++i) {
		//Whatever particles haven't been resampled, are just randomly scattered to explore.
		particles[i] = new Particle();
		particles[i].randomize();
	}
}
function noisifyParticles() {
	for(var i=0; i<particles.length; ++i) {
		particles[i].pos[0] += randomNormal(0, particlePosNoiseVariance);
		particles[i].pos[1] += randomNormal(0, particlePosNoiseVariance);
		particles[i].orien += randomNormal(0, particleOrientationNoiseVariance);
	}
}
function propogateParticles(dPos, dOrien) {
	if(dPos[0] != 0 || dPos[1] != 0 || dOrien != 0) {
		for(var i=0; i<particles.length; ++i) {
			var randomDist = randomNormal(0, odomPosNoiseVar);
			var randomAngle = Math.random() * Math.PI;
			dPos[0] += randomDist * Math.cos(randomAngle);
			dPos[1] += randomDist * Math.sin(randomAngle);
			dOrien += randomNormal(0, odomOrienNoiseVar);
			particles[i].pos[0] += dPos[0];
			particles[i].pos[1] += dPos[1];
			particles[i].orien += dOrien;
		}
	}
}

function randomNormal(mu, sigma) {
	//Computed using the Box-Muller transform.
	//https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
	var u1 = Math.random();
	var u2 = Math.random();
	var z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
	return mu + (z0 * sigma);
}
function weightToColor(weight) {
	//This is kinda gross. Just ignore it.
	//In the future, go steal a colormap function.
	if(weight > 1) {
		weight = 1;
	}

	//Create HSL
	var h = ((1-weight)*240)/360;
	var s = 1;
	var l = 0.5;

	//Convert to RGB (from https://gist.github.com/mjackson/5311256)
	var r, g, b;

	function hue2rgb(p, q, t) {
		if (t < 0) t += 1;
		if (t > 1) t -= 1;
		if (t < 1/6) return p + (q - p) * 6 * t;
		if (t < 1/2) return q;
		if (t < 2/3) return p + (q - p) * (2/3 - t) * 6;
		return p;
	}

	var q = l < 0.5 ? l * (1 + s) : l + s - l * s;
	var p = 2 * l - q;

	r = hue2rgb(p, q, h + 1/3);
	g = hue2rgb(p, q, h);
	b = hue2rgb(p, q, h - 1/3);

	r = Math.floor(r * 255);
	g = Math.floor(g * 255);
	b = Math.floor(b * 255);

	//Convert to RGB color code
	function componentToHex(c) {
		var hex = c.toString(16);
		return hex.length == 1 ? "0" + hex : hex;
	}
	function rgbToHex(r, g, b) {
		return "#" + componentToHex(r) + componentToHex(g) + componentToHex(b);
	}

	return rgbToHex(r, g, b);
}
function errorColor(error) {
	//Restrict the error weight to [0,1], and get its corresponding color.
	var errorWeight = error / errorWeightColorDivisor;
	if(errorWeight < 0) { errorWeight = 0; }
	if(errorWeight > 1) { errorWeight = 1; }
	errorWeight = 1 - errorWeight;
	return weightToColor(errorWeight);
}
function dist2(a, b) {
	//Euclidean distance-squared.
	//Often used instead of regular Euclidean distance, since computing a square root is expensive and usually unnecessary.
	var dx = b[0]-a[0];
	var dy = b[1]-a[1];
	return (dx*dx) + (dy*dy);
}
function angleDist(a, b) {
	//Compute the "distance" between two angles.
	var diff = a - b;
	function specialMod(lhs, rhs) {
		return lhs - (Math.floor(lhs/rhs) * rhs);
	}
	return (specialMod(diff + Math.PI, Math.PI*2)) - Math.PI;
}
function mean(arr) {
	//Compute the mean of a given array.
	var total = 0;
	for(var i=0; i<arr.length; ++i) {
		total += arr[i];
	}
	return total / arr.length;
}
function variance(arr) {
	//Compute the variance of a given array.
	var v = 0;
	var m = mean(arr);
	for(var i=0; i<arr.length; ++i) {
		v += arr[i]*arr[i];
	}
	v /= arr.length;
	v -= m*m;
	return v;
}
function normalizeWeight(arr) {
	//Normalize an array so its elements sum to 1.
	var total = 0;
	for(var i=0; i<arr.length; ++i) {
		total += arr[i];
	}
	for(var i=0; i<arr.length; ++i) {
		arr[i] /= total;
	}
	return arr;
}
function cumsum(arr) {
	//Take the cumulative sum of an array.
	for(var i=1; i<arr.length; ++i) {
		arr[i] += arr[i-1];
	}
	return arr;
}
function weightFromDistance(distances) {
	//Given an array of particle LIDAR distance differences, convert it to a list of weights.
	var v = variance(distances);
	var m = 1/(Math.sqrt(2*Math.PI*v));
	var weights = [];
	for(var i=0; i<distances.length; ++i) {
		weights[i] = Math.pow(Math.E, -(Math.pow((distances[i]), 2) / (2*v))) * m;
	}
	return weights;
}
function getProbFromLog(l) {
	//
	return (1 / (1 + Math.exp(l)));
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);