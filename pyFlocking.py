#Processing.Py translation of Flocking
#Translated by Joshua Kingsbury

#http://processing.org/examples/flocking.html

## Flocking
## Daniel Shiffman <http://www.shiffman.net>
## The Nature of Code, Spring 2009

## Demonstration of Craig Reynolds' "Flocking" behavior
## See: http://www.red3d.com/cwr/
## Rules: Cohesion, Separation, Alignment

## Click mouse to add boids into the system

flock = None

def setup():
	global flock
	size(500,500)
	flock = Flock()
	## Add an initial set of boids into the system
	for i in range(100):
		flock.addBoid(

			Boid( PVector(width/2,height/2), 3.0, 0.05 )
			)
	smooth()

def draw():
	background(255)
	flock.run()

## Add a new boid into the System

def mousePressed():
	flock.addBoid(
		Boid( PVector(mouseX, mouseY), 2.0, 0.05 ))

## Boid class
## Methods for Separation, Cohesion, Alignment added

class Boid:
	def __init__(self,l, ms, mf):
		self.acc = PVector()
		self.vel = PVector(random(-1,1), random(-1,1))
		self.loc = l
		self.r = 2.0
		self.maxspeed = ms
		self.maxforce = mf

	def run(self,boids):
		self.flock(boids)
		self.update()
		self.borders()
		self.render()

	## We accumulate a new acceleration each time based on three rules
	def flock(self,boids):
		sep = self.seperate(boids) ##Seperation
		ali = self.align(boids)    ##Alignment
		coh = self.cohesion(boids) ##Cohesion
		## Arbitrarily weight these forces
		sep.mult(1.5)
		ali.mult(1.0)
		coh.mult(1.0)
		## Add the force vectors to acceleration
		self.acc.add(sep)
		self.acc.add(ali)
		self.acc.add(coh)

	## Method to update location
	def update(self):
		## Update velocity
		self.vel.add(self.acc)
		## Limit speed
		self.vel.limit(self.maxspeed)
		self.loc.add(self.vel)
		## Reset accelertion to 0 each cycle
		self.acc.mult(0)

	def seek(self,target):
		self.acc.add(self.steer(target,False))

	def arrive(self,target):
		self.acc.add(self.steer(target,True))

	## A method that calculates a steering vector towards a target
	## Takes a second argument, if true, it slows down as it approaches the target
	def steer(self,target,slowdown):
		steer = PVector()  ## The steering vector
		desired = PVector.sub(target,self.loc)  ## A vector pointing from the location to the target
		d = desired.mag()  ## Distance from the target is the magnitude of the vector
		## If the distance is greater than 0, calc steering (otherwise return zero vector)
		if d > 0:
			## Normalize desired
			desired.normalize()
			## Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
			if slowdown and d < 100.0:
				desired.mult(self.maxspeed*(d/100.0))  ## This damping is somewhat arbitrary
			else:
				desired.mult(self.maxspeed)
			## Steering = Desired minus Velocity
			steer = PVector.sub(desired, self.vel)
			steer.limit(self.maxforce)  ## Limit to maximum steering force
		else:
			steer = PVector(0,0)
		return steer

	def render(self):
		## Draw a triangle rotated in the direction of velocity
		theta = self.vel.heading2D() + radians(90)
		fill(175)
		stroke(0)
		pushMatrix()
		translate(self.loc.x,self.loc.y)
		rotate(theta)
		beginShape(TRIANGLES)
		vertex(0, -self.r*2)
		vertex(-self.r, self.r*2)
		vertex(self.r, self.r*2)
		endShape()
		popMatrix()

	##Wraparound
	def borders(self):
		if (self.loc.x < -self.r): 
			self.loc.x = width+self.r
		if (self.loc.y < -self.r): 
			self.loc.y = height+self.r
		if (self.loc.x > width+self.r): 
			self.loc.x = -self.r
		if (self.loc.y > height+self.r): 
			self.loc.y = -self.r

	## Separation
	## Method checks for nearby boids and steers away
	def seperate(self,boids):
		desiredseperation = 25.0
		steer = PVector(0,0,0)
		count = 0.0
		## For every boid in the system, check if it's too close
		for i, j in enumerate(boids):
			other = j
			d = PVector.dist(self.loc, other.loc)
			## If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
			if d > 0 and d < desiredseperation:
				## Calculate vector pointing away from neighbor
				diff = PVector.sub(self.loc,other.loc)
				diff.normalize()
				diff.div(d)  ## Weight by distance
				steer.add(diff)
				count+=1  ## Keep track of how many
		## Average -- divide by how many
		if count > 0:
			steer.div(count)
		## As long as the vector is greater than 0
		if steer.mag() > 0:
			## Implement Reynolds: Steering = Desired - Velocity
			steer.normalize()
			steer.mult(self.maxspeed)
			steer.sub(self.vel)
			steer.limit(self.maxforce)
		return steer

	def align(self,boids):
		neighbordist = 50.0
		steer = PVector(0,0,0)
		count = 0.0
		for i,j in enumerate(boids):
			d = PVector.dist(self.loc,j.loc)
			#Find combined velocity of neighboring boids
			if d > 0 and d < neighbordist:
				steer.add(j.vel)
				count+=1
		#find avg velocity of all boids in neighbordist
		if count > 0:
			steer.div(count)

		## As long as the vector is greater than 0
		if steer.mag() > 0:
			## Implement Reynolds: Steering = Desired - Velocity
			steer.normalize()
			steer.mult(self.maxspeed)
			steer.sub(self.vel)
			steer.limit(self.maxforce)

		return steer

	## Cohesion
	## For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
	def cohesion(self,boids):
		neighbordist = 50.0
		sum = PVector(0,0,0)  ## Start with empty vector to accumulate all locations
		count = 0.0
		for i,j in enumerate(boids):
			d = PVector.dist(self.loc,j.loc)
			if d > 0 and d < neighbordist:
				sum.add(j.loc)  ## Add location
				count+=1
		if count > 0:
			sum.div(count)
			return self.steer(sum,False)  ## Steer towards the location
		return sum


## Flock class
## Does very little, simply manages the ArrayList of all the boids

class Flock:
	def __init__(self):
		self.boids = []

	def run(self):
		for i,j in enumerate(self.boids):
			j.run(self.boids)

	def addBoid(self,boid):
		self.boids.append(boid)