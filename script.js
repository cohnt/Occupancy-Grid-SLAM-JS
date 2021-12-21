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
var robotSpeed = 1; // Robot speed, in meters per second
var robotTurnRate = 180 * (Math.PI / 180); // Robot turn rate, in radians per second

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
function drawFrame() {
	clearCanvas(worldCtx);

	//Draw the obstacles onto the world
	for(var i=0; i<obstacles.length; ++i) {
		obstacles[i].draw(worldCtx);
	}
	//Draw the robot onto the world
	drawRobot(worldCtx, robotPose);
}

function reset() {
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
function magnitude(v) {
	var total = 0;
	for(var i=0; i<v.length; ++i) {
		total += Math.pow(v[i], 2);
	}
	return Math.sqrt(total);
}
function segmentLength(segment) {
	//
	return distance(segment[0], segment[1]);
}
function distance(p1, p2) {
	var total = 0;
	for(var i=0; i<p1.length; ++i) {
		total += Math.pow(p1[i] - p2[i], 2);
	}
	return Math.sqrt(total);
}
function vectorDot(v, w) {
	var total = 0;
	for(var i=0; i<v.length; ++i) {
		total += v[i] * w[i];
	}
	return total;
}
function lineCircleCollisionTest(segment, circleCenter, radius) {
	// Get distance from each end of the segment to the circle center
	var p1Distance = distance(segment[0], circleCenter);
	var p2Distance = distance(segment[1], circleCenter);
	if((p1Distance <= radius) != (p2Distance <= radius)) {
		//If one point is inside the circle and the other is outside, then they intersect
		return true;
	}
	if(p1Distance <= radius && p2Distance <= radius) {
		//If both points are inside the circle, then they don't intersect
		return false;
	}

	var v = [circleCenter[0] - segment[0][0], circleCenter[1] - segment[0][1]];
	var vLine = [[0, 0], v];
	var vMag = magnitude(v);
	var vUnitVec = [v[0]/vMag, v[1]-vMag];

	var segmentMag = segmentLength(segment);
	var segmentUnitVec = [
		(segment[1][0]-segment[0][0])/segmentMag,
		(segment[1][1]-segment[0][1])/segmentMag
	];
	var rVec = [
		segmentUnitVec[1] * radius * -1,
		segmentUnitVec[0] * radius
	];
	var rLine = [
		[
			rVec[0] + radius,
			rVec[1] + radius
		],
		[
			radius + (vUnitVec[1] * radius * -1),
			radius + (vUnitVec[0] * radius)
		]
	];

	compVontoR = Math.abs(vectorDot(rVec, v)/radius);

	if(compVontoR <= radius) {
		return lineCollisionTest(rLine, segment);
	}
	else {
		return false;
	}
}
function lineCollisionTest(l1, l2) {
	var a, b, c, d, e, f, g, h, u, v, p, m, t;
	//Special case: line segments are on the same line
	if(Math.abs(l1[0] / magnitude(l1)) == Math.abs(l2[0] / magnitude(l2))) {
		//Ok, they are parallel
		p = [l2[0][0]-l1[0][0], l2[0][1]-l1[0][1]];
		if(p[0] == 0 && p[1] == 0) {
			return true; //they contain the same endpoint
		}
		//Ok, p isn't a common endpoint
		m = magnitude(p);
		p[0] /= m;
		p[1] /= m;
		if(Math.abs(p[0]) != Math.abs(l1[0] / magnitude(l1))) {
			return false;
		}

		//Ok, they are along the same line...
		if(l1[0][0] >= l2[0][0] && l1[0][0] <= l2[1][0]) {
			return true;
		}
		if(l1[0][0] <= l2[0][0] && l1[0][0] >= l2[1][0]) {
			return true;
		}
		if(l1[1][0] >= l2[0][0] && l1[1][0] <= l2[1][0]) {
			return true;
		}
		if(l1[1][0] <= l2[0][0] && l1[1][0] >= l2[1][0]) {
			return true;
		}
	}

	//l1=<1x1+(1x2-1x1)u,1y1+(1y2-1y1)u>
	//l2=<2x1+(2x2-2x1)v,2y1+(2y2-2y1)v>

	//{1x1+(1x2-1x1)u = 2x1+(2x2-2x1)v
	//{1y1+(1y2-1y1)u = 2y1+(2y2-2y1)v
	//  or
	//{a+bu = c+dv
	//{e+fu = g+hv
	//  becomes
	//u = (hc+de-dg-ha)/(hb-df)
	//v = (fa+bg-be-fc)/(fd-bh)
	//if u and v are between 0 and 1, they intersect
	a = l1[0][0];
	b = l1[1][0] - l1[0][0];
	c = l2[0][0];
	d = l2[1][0] - l2[0][0];
	e = l1[0][1];
	f = l1[1][1] - l1[0][1];
	g = l2[0][1];
	h = l2[1][1] - l2[0][1];

	u = ((h*c)+(d*e)-(d*g)-(h*a))/((h*b)-(d*f));
	v = ((f*a)+(b*g)-(b*e)-(f*c))/((f*d)-(b*h));

	if((u>=0) && (u<=1) && (v>=0) && (v<=1)) {
		return true;
	}
	else {
		return false;
	}
}

/////////////////////
/// EXECUTED CODE ///
/////////////////////

requestAnimationFrame(setup);