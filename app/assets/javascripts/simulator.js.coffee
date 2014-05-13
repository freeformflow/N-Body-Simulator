# This is the "naive" version of the 2D N-Body simulator.

# Create an Object container that holds functions and variables.
window.View = {}
window.Engine = {}



#-------------------------------------------------------------------------------
# This block contains all code for the view that displays the simulation.
#-------------------------------------------------------------------------------

# Initialize the canvas view.
View.InitializeViewport = () ->
  View.background_color = '#87CEFA'
  View.body_core_color = '#1874CD'
  View.body_size = 3

  View.canvas = document.getElementById 'view'
  View.ctx = View.canvas.getContext '2d'
  #View.ctx.fillStyle = View.background_color
  #View.ctx.fillRect 0, 0, View.canvas.width, View.canvas.height

  # Also, we need to seed the random number generator, so we may place particles
  # at the simulation start.
  d = new Date()
  Engine.seed = d.getTime()

  # Other Engine variable that need to be set.  Reorganize this later.
  Engine.n = 200
  Engine.dt = 0.1
  Engine.run = true
  Engine.initial_speed = 20


# This function produces animation by updating the view when we repeatedly call it.
View.Update = () ->

  # First, wipe the view.
  View.ctx.clearRect 0, 0, View.canvas.width, View.canvas.height

  # Now, create a new background.
  View.ctx.fillStyle = View.background_color
  View.ctx.fillRect 0, 0, View.canvas.width, View.canvas.height

  # Finally, draw all the particles.
  for i in [0..Engine.n - 1]
    View.RenderBody Engine.pos_x[i], Engine.pos_y[i]



# This function draws a single particle.  It will be called frequently, haha.
View.RenderBody = (x, y) ->
  View.ctx.fillStyle = View.body_core_color
  View.ctx.beginPath()
  View.ctx.moveTo(x, y)
  View.ctx.arc(x, y, View.body_size, 0, 2*Math.PI, true) # Draws a circle.
  View.ctx.fill()


#-------------------------------------------------------------------------------
# This block contains all code for the simulation engine.
#-------------------------------------------------------------------------------

Engine.Main = () ->
  # This is the main application loop.

  foo = true

  Engine.Initialize()

  while foo
    Engine.IntegrateDynamics()
    Engine.ApplyConstraints()
    View.Update()
    alert "Continue closing this until you can ask the browser to not display it any more.  There is an animation bug I'm still working on."

Engine.Initialize = () ->
  View.InitializeViewport()
  Engine.SeedSimulation()

Engine.IntegrateDynamics = () ->
  # Dynamics are achieved using Velocity-Verlet Integration.
  # Step 1: pos(t + dt)       = pos(t) + vel(t) * dt + 0.5 * acc(t) * dt**2
  # Step 2: vel(t + 0.5 * dt) = vel(t) + 0.5 * acc(t) * dt
  # Step 3: acc(t + dt)       = Force Evaluation / mass  @time = t + dt
  # Step 4: vel(t + dt)       = vel(t + 0.5 * dt) + 0.5 * acc(t + dt) * dt

  # Step 1:
  for i in [0..Engine.n - 1]
    Engine.pos_x[i] = Engine.pos_x[i] + Engine.vel_x[i] * Engine.dt +
                      0.5 * Engine.acc_x[i] * Engine.dt * Engine.dt

    Engine.pos_y[i] = Engine.pos_y[i] + Engine.vel_y[i] * Engine.dt +
                  0.5 * Engine.acc_y[i] * Engine.dt * Engine.dt
  # Step 2:
  for i in [0..Engine.n - 1]
    Engine.vel_x[i] = Engine.vel_x[i] + 0.5 * Engine.acc_x[i] * Engine.dt

    Engine.vel_y[i] = Engine.vel_y[i] + 0.5 * Engine.acc_y[i] * Engine.dt

  # Step 3:
  Engine.ForceEvaluation()

  # Step 4:
  for i in [0..Engine.n - 1]
    Engine.vel_x[i] = Engine.vel_x[i] + 0.5 * Engine.acc_x[i] * Engine.dt
    Engine.vel_y[i] = Engine.vel_y[i] + 0.5 * Engine.acc_y[i] * Engine.dt



Engine.ForceEvaluation = () ->
  # This function performs a force evaluation for the system.  For a single particle,
  # it determines the sum of the force exerted by all other particles.  It then
  # repeats the process for all particles.

  for i in [0..Engine.n - 1]
    Engine.acc_x[i] = 0
    Engine.acc_y[i] = 0

  for i in [1..Engine.n - 1]
    for j in [0..i - 1]

      # Start by geting the distance between i and j
      x = Engine.pos_x[i] - Engine.pos_x[j]
      y = Engine.pos_y[i] - Engine.pos_y[j]
      dist = Math.sqrt(x*x + y*y)

      # Use this distance to get the force between i and j
      # Force = -(dE/dr), which for gravitation is just (G * m_i * m_j) / distance**2
      # We can ignore a lot those terms since we are working with reduced units.

      if dist > 20
        force = 1000 / (dist * dist)

        Engine.acc_x[i] = Engine.acc_x[i] - force * (x / dist)
        Engine.acc_y[i] = Engine.acc_y[i] - force * (y / dist)

        Engine.acc_x[j] = Engine.acc_x[j] + force * (x / dist)
        Engine.acc_y[j] = Engine.acc_y[j] + force * (y / dist)


Engine.ApplyConstraints = () ->
  # Apply constraints.  These will keep the particles within the box, because if
  # they get loose, there is no force to bring them back.
  for i in [0..Engine.n - 1]

    if Engine.pos_x[i] < 0
      # particle has gone off the left edge.
      Engine.pos_x[i] = 0
      Engine.vel_x[i] = 0
      Engine.acc_x[i] = 0

    if Engine.pos_x[i] > View.canvas.width
      # particle has gone off the right edge.
      Engine.pos_x[i] = View.canvas.width
      Engine.vel_x[i] = 0
      Engine.acc_x[i] = 0

    if Engine.pos_y[i] < 0
      # particle has gone off the top edge.
      Engine.pos_y[i] = 0
      Engine.vel_y[i] = 0
      Engine.acc_y[i] = 0

    if Engine.pos_y[i] > View.canvas.height
      # particle has gone off the bottom edge.
      Engine.pos_y[i] = View.canvas.width
      Engine.vel_y[i] = 0
      Engine.acc_y[i] = 0

Engine.SeedSimulation = () ->
  # This function seeds the simulation with particles.  It allocates space for
  # their properties (position, velocity, acceleration) in memory, and seeds
  # their initial position through the use of a random number generator.

  Engine.pos_x = []
  Engine.pos_y = []
  Engine.vel_x = []
  Engine.vel_y = []
  Engine.acc_x = []
  Engine.acc_y = []

  for i in [0..Engine.n - 1]
    Engine.pos_x.push Engine.RandomNumber() * View.canvas.width
    Engine.pos_y.push Engine.RandomNumber() * View.canvas.height

    x = Engine.pos_x[i] - 300
    y = Engine.pos_y[i] - 300
    dist = Math.sqrt(x*x + y*y)

    if dist > 10
      Engine.vel_x.push  Engine.initial_speed * (y / dist)
      Engine.vel_y.push  -1 * Engine.initial_speed * (x / dist)
    else
      Engine.vel_x.push 0
      Engine.vel_y.push 0

    Engine.acc_x.push 0
    Engine.acc_y.push 0


Engine.RandomNumber = () ->
  # This is the random number generator.  Here we implement it
  # by linking two linear congruential generators.  This function
  # returns a number between 0 and 1, inclusive.

  A =  (1664525 * Engine.seed + 1013904223) % 4294967296
  Engine.seed =  (1103515245 * A + 12345) % 2147483648

  return Engine.seed / 2147483648
