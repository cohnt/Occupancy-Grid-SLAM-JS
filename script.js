/////////////////
/// CONSTANTS ///
/////////////////

var canvasSize = [0.49, 0.7]; //Size of each canvas, as a fraction of the total page width and height, respectively
var worldScale = 20; //The width of the entire browser viewport (in meters), determining the scale of the displays
var canvasLineWidth = 0.015;
var robotRadius = 0.2;
var robotMarkerTriangleAngle = 30 * (Math.PI / 180); //The front angle of the triangular robot marker
var robotStrokeStyle = "black";
var obstacleStrokeStyle = "black";
var lidarStrokeStyle = "red";
var obstacleSizeRange = [0.5, 2];
var numObstacles = 15;
var robotSpeed = 0.5; // Robot speed, in meters per second
var robotTurnRate = 90 * (Math.PI / 180); // Robot turn rate, in radians per second
var lidarNumPoints = 50; // Number of points given in each sweep of the lidar
var lidarFOV = 360 * (Math.PI / 180); // FOV of the lidar, in radians
var lidarAngle = lidarFOV / (lidarNumPoints - 1); // The angle between two lidar beams
var lidarNoiseVariance = 0.05; //The variance of the noise affecting the lidar measurements, in meters.

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

var pixelsPerMeter; //Pixels per meter
var worldWidth; //World width in meters
var worldHeight; //World height in meters
var worldMaxX; //The maximum x coordinate shown in the world, i.e., worldWidth/2
var worldMaxY; //The maximum y coordinate shown in the world, i.e., worldHeight/2

var obstacles = []; //A list of obstacle objects
var obstacleSegments = []; //A list of all segments of all obstacles
var robotPose;
var lastFrameTime;
var lidarDistances = []; //When simulating what the LIDAR sensor would see, this contains all of the distance readings.
var lidarEnds = []; //The endpoints of each LIDAR beam.

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

	pixelsPerMeter = window.innerWidth / worldScale;
	worldWidth = canvasSize[0] * window.innerWidth / pixelsPerMeter;
	worldHeight = canvasSize[1] * window.innerHeight / pixelsPerMeter;
	worldMaxX = worldWidth / 2;
	worldMaxY = worldHeight / 2;

	// Create the canvas contexts
	worldCtx = worldCanvas.getContext("2d");
	mapCtx = worldCanvas.getContext("2d");

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
		reset(); //Get everything back to its initial state.
	}
}
function newWorldButtonClick() {
	//This is the callback function if you click the new world button.
	if(!running) {
		//If we aren't currently running, generate a new world, and reset.
		//resetContext() isn't needed, because the canvas itself isn't changing.
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
	ctx.stroke();
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

	//Draw the obstacles onto the world
	for(var i=0; i<obstacles.length; ++i) {
		obstacles[i].draw(worldCtx);
	}
	//Draw the robot onto the world
	drawRobot(worldCtx, robotPose);
	drawLidar(worldCtx);
}

function reset() {
	lidarDistances = [];
	lidarEnds = [];
	robotPose = new Pose([0, 0], 0);
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
	computeLidarDists(robotPose);
	noisifyLidar();
	computeLidarEndpoints(robotPose);

	drawFrame();

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
	lidarDistances = [];
	lidarEnds = [];
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
		var bestDist = Infinity;
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
		lidarDistances.push(bestDist);
	}
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
		lidarEnds.push([
			pose.pos[0] + (lidarDistances[i] * Math.cos(globalFrameAngle)),
			pose.pos[1] + (lidarDistances[i] * Math.sin(globalFrameAngle))
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
function randomNormal(mu, sigma) {
	//Computed using the Box-Muller transform.
	//https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
	var u1 = Math.random();
	var u2 = Math.random();
	var z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
	return mu + (z0 * sigma);
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);