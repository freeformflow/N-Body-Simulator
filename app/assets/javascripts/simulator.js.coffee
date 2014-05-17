# This is the "naive" version of the 2D N-Body simulator.

# Create an Object container that holds functions and variables.
window.View = {}
window.Engine = {}

Engine.run = false

#-------------------------------------------------------------------------------
# This block contains all code for the view that displays the simulation.
#-------------------------------------------------------------------------------

View.SizeCanvas = () ->
  # This function will size the canvas according how the User is viewing the page.
  # The goal is to have a responsive layout, including support for mobile viewing.
  w = window.innerWidth
  h = window.innerHeight

  if w > h
    document.getElementById('view').width = 0.95 * w
    document.getElementById('view').height = 0.95 * (h - 200)
    View.body_size = h / 300

  else
    document.getElementById('view').width = 0.95 * w
    document.getElementById('view').height = 0.95 * w
    View.body_size = w / 250


  # Now set the size of the Settings interface overlay.
  if w > h
    document.getElementById('settings_overlay').width = 0.95 * w
    document.getElementById('settings_overlay').height = 0.95 * (h - 200)
  else
    document.getElementById('settings_overlay').width = 0.95 * w
    document.getElementById('settings_overlay').height = 0.95 * w

  document.getElementById('settings').style.display = 'none'


  # Now we can measure the canvas's offset from the edges of the window.
  # We'll need these values to figure out the cursor's location in simulation.
  Engine.reference_x = document.getElementById('settings_overlay').offsetLeft
  Engine.reference_y = document.getElementById('settings_overlay').offsetTop

  Engine.user_x = null
  Engine.user_y = null



# Initialize the canvas view.
View.InitializeViewport = () ->
  View.background_color = '#0066FF'
  View.body_core_color = '#003380'


  View.canvas = document.getElementById 'view'
  View.ctx = View.canvas.getContext '2d'


# This function produces animation by updating the view when we repeatedly call it.
View.Update = () ->

  # We need to carefully time the animation loops.  We start our stopwatch now and
  # check it when we're done drawing.
  render_start_time = new Date().getTime()

  # Begin drawing.... First, wipe the view.
  View.ctx.clearRect 0, 0, View.canvas.width, View.canvas.height

  # Now, create a new background.
  View.ctx.fillStyle = View.background_color
  View.ctx.fillRect 0, 0, View.canvas.width, View.canvas.height

  # Update the dynamics using the simulator's Engine
  Engine.IntegrateDynamics()
  Engine.ApplyConstraints()

  # Finally, draw all the particles.
  for i in [0..Engine.n - 1]
    View.RenderBody Engine.pos_x[i], Engine.pos_y[i]

  # We're done drawing.  Check the stopwatch now.
  elapse = new Date().getTime() - render_start_time

  # Rinse and repeat...  This recursive will call itself indefinitely.
  callback = -> View.Update()

  # We need at least 25 frames per second to give realistic motion.
  # That is 40 ms per frame.  If we're lucky, we have time to spare, and
  # we can ask the CPU to idle.  Otherwise, move on to the next frame right away,
  # but lag will be noticeable...  :(
  if elapse < 40
    View.render_id = window.setTimeout callback, 40 - elapse
  else
    View.render_id = window.setTimeout callback, 0




# This function draws a single particle.  It will be called frequently, haha.
View.RenderBody = (x, y) ->
  View.ctx.fillStyle = View.body_core_color
  View.ctx.beginPath()
  View.ctx.moveTo(x, y)
  View.ctx.arc(x, y, View.body_size, 0, 2*Math.PI, true) # Draws a circle.
  View.ctx.fill()

View.ShowSettings = () ->

  if document.getElementById('settings').style.display == 'none'
    document.getElementById('settings').style.display = 'block'
  else
    document.getElementById('settings').style.display = 'none'




#-------------------------------------------------------------------------------
# This block contains all code for the simulation engine.
#-------------------------------------------------------------------------------

Engine.Initialize = () ->
  # There are certain things that need to happen before we can get started.

  # Seed the random number generator.  This generator lets us scatter particles evenly
  # at the simulation start.
  d = new Date()
  Engine.seed = d.getTime()

  # Other Engine parameters need to be set.  These control the performance
  # and physics within the simulation.  All of them can be manipulated by the
  # User via the User interface.
  Engine.n = document.getElementById('n_control').value
  Engine.dt = document.getElementById('dt_control').value
  Engine.run = true
  Engine.initial_speed = document.getElementById('Vi_control').value
  Engine.G = document.getElementById('G_control').value
  Engine.wall = document.getElementById('wall_control').value
  Engine.user_input = document.getElementById('input_control').checked
  Engine.sticky = document.getElementById('sticky_control').checked

  # Now we can get started.  Update() is a recursive function that runs the simulation.
  View.InitializeViewport()
  Engine.SeedSimulation()
  View.Update()

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
  # This function performs a force evaluation for the system.  The basic
  # algorithm is to take a single particle and determine the sum of the force
  # exerted by all other particles. Then repeat the process for all particles.

  # Zero out every particle's acceleration.
  for i in [0..Engine.n - 1]
    Engine.acc_x[i] = 0
    Engine.acc_y[i] = 0

  Engine.collide_i = []
  Engine.collide_j = []
  Engine.collide_x = []
  Engine.collide_y = []
  Engine.collide_idist = []

  # We can more efficently calculat this pair-wise sum by doing it for only the
  # Upper Triangle of the pair matrix.  This way we only calculate the distance
  # between each pair only once.
  for i in [1..Engine.n - 1]
    for j in [0..i - 1]

      # Start by geting the distance between i and j
      x = Engine.pos_x[i] - Engine.pos_x[j]
      y = Engine.pos_y[i] - Engine.pos_y[j]
      dist_sq = x*x + y*y
      dist_inverse = 1 / Math.sqrt(dist_sq)

      # Use this distance to get the force between i and j
      # Force = -(dE/dr), which for gravitation is (G * m_i * m_j) / distance**2
      # We can ignore a lot those terms since we are working with reduced units
      # and are cheating to get better visuals.

      # Only apply this to particles that are far enough apart. Otherwise, the
      # particles sling themselves past one another.
      if dist_sq > 400
        force = Engine.G / dist_sq

        # This force is applied to i.
        Engine.acc_x[i] = Engine.acc_x[i] - force * x * dist_inverse
        Engine.acc_y[i] = Engine.acc_y[i] - force * y * dist_inverse

        # Now the opposite and equal force is applied to j.
        Engine.acc_x[j] = Engine.acc_x[j] + force * x * dist_inverse
        Engine.acc_y[j] = Engine.acc_y[j] + force * y * dist_inverse

      else if Engine.sticky and dist_sq < 12 and dist_sq > 1
        # If the User has requested particle-particle collisions, we note any overlapping
        # particles here.  We address these pairs inside Engine.ApplyConstraints()

        Engine.collide_i.push i
        Engine.collide_j.push j
        Engine.collide_x.push x
        Engine.collide_y.push y
        Engine.collide_idist.push dist_inverse


  # Now, if the user has opted to use the cursor as an attractor of particles,
  # we calculate those forces here.
  if Engine.user_input and Engine.user_x != null
    for i in [0..Engine.n - 1]
      x = Engine.pos_x[i] - Engine.user_x
      y = Engine.pos_y[i] - Engine.user_y
      dist_sq = x*x + y*y
      dist_inverse = 1 / Math.sqrt(dist_sq)

      if dist_sq > 400
        force = 100000 / dist_sq

        Engine.acc_x[i] = Engine.acc_x[i] - force * x * dist_inverse
        Engine.acc_y[i] = Engine.acc_y[i] - force * y * dist_inverse



Engine.ApplyConstraints = () ->
  # Apply constraints.  These will keep the particles within the box, because if
  # they get loose, there is not enough force to bring them back.  We can also use the
  # walls to bleed energy out of the system and make it behave better.  If a
  # particle goes past a wall, it is put back within bounds and its velocity
  # in that direction may be modified.

  for i in [0..Engine.n - 1]

    if Engine.pos_x[i] < 0
      # particle has gone off the left edge.
      Engine.pos_x[i] = 0
      Engine.vel_x[i] = -Engine.wall * Engine.vel_x[i]
      Engine.acc_x[i] = 0

    else if Engine.pos_x[i] > View.canvas.width
      # particle has gone off the right edge.
      Engine.pos_x[i] = View.canvas.width
      Engine.vel_x[i] = -Engine.wall * Engine.vel_x[i]
      Engine.acc_x[i] = 0

    else if Engine.pos_y[i] < 0
      # particle has gone off the top edge.
      Engine.pos_y[i] = 0
      Engine.vel_y[i] = -Engine.wall * Engine.vel_y[i]
      Engine.acc_y[i] = 0

    else if Engine.pos_y[i] > View.canvas.height
      # particle has gone off the bottom edge.
      Engine.pos_y[i] = View.canvas.height
      Engine.vel_y[i] = -Engine.wall * Engine.vel_y[i]
      Engine.acc_y[i] = 0

    if Engine.sticky
      # If the User has requested particle-particle collisions, we calculate here.
      # We model these collisions as inelastic, meaning only momentum is conserved.

      for k in [0..Engine.collide_i.length - 1]

        i = Engine.collide_i[k]
        j = Engine.collide_j[k]
        x = Engine.collide_x[k]
        y = Engine.collide_y[k]
        dist_inverse = Engine.collide_idist[k]

        # Move the particles so they are no longer overlapping.
        Engine.pos_x[i] = Engine.pos_x[j] + 1.2 * x
        Engine.pos_y[i] = Engine.pos_y[j] + 1.2 * y

        # Modify both particle's velocity.  Because they have the same mass
        # their final velcities are equal and the average of their inital velocities.
        # m_i * v_i  + m_j * v_j  = M * V   =>  V = (v_i + v_j) / 2 
        Engine.vel_x[i] = Engine.vel_x[j] =
            0.5 * (Engine.vel_x[i] + Engine.vel_x[j])

        Engine.vel_y[i] = Engine.vel_y[j] =
            0.5 * (Engine.vel_y[i] + Engine.vel_y[j])


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

    x = Engine.pos_x[i] - View.canvas.width / 2
    y = Engine.pos_y[i] - View.canvas.height / 2
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
  # This is a random number generator.  Here, we implement it
  # by linking two linear congruential generators.  This function
  # returns a number between 0 and 1, inclusive.

  A =  (1664525 * Engine.seed + 1013904223) % 4294967296
  Engine.seed =  (1103515245 * A + 12345) % 2147483648

  return Engine.seed / 2147483648


Engine.GetMousePos = (event) ->
  Engine.user_x = event.clientX
  Engine.user_y = event.clientY

  Engine.user_x = Engine.user_x - Engine.reference_x
  Engine.user_y = Engine.user_y - Engine.reference_y




#-------------------------------------------------------------------
# This code block handles User controls and updates settings here.
#-------------------------------------------------------------------

Engine.Begin = () ->
  document.getElementById('pause_button').style.display = 'inline';
  document.getElementById('play_button').style.display = 'none';
  document.getElementById('start_button').onclick = -> window.Engine.Pause()
  Engine.Initialize()

Engine.Pause = () ->
  window.clearTimeout View.render_id
  document.getElementById('play_button').style.display = 'inline';
  document.getElementById('pause_button').style.display = 'none';
  document.getElementById('start_button').onclick = -> window.Engine.Continue()

Engine.Continue = () ->
  document.getElementById('pause_button').style.display = 'inline';
  document.getElementById('play_button').style.display = 'none';
  document.getElementById('start_button').onclick = -> window.Engine.Pause()
  View.Update()

Engine.Reset = () ->
  window.clearTimeout View.render_id
  Engine.Begin()



Engine.ChangeN = () ->
  if document.getElementById('n_control').value < Engine.n
    Engine.n = document.getElementById('n_control').value
  else
    Engine.n = document.getElementById('n_control').value
    Engine.Reset()

Engine.ChangeG = () ->
  Engine.G = document.getElementById('G_control').value

Engine.Changedt = () ->
  Engine.dt = document.getElementById('dt_control').value

Engine.ChangeWall = () ->
  Engine.wall = document.getElementById('wall_control').value

Engine.ChangeInput = () ->
  box = document.getElementById('input_control')

  if box.checked
    Engine.user_input = true
    document.getElementById('settings_overlay').onmousemove = -> window.Engine.GetMousePos(event)
  else
    Engine.user_input = false
    document.getElementById('settings_overlay').onmousemove = null

Engine.ChangeSticky = () ->
  box = document.getElementById('sticky_control')

  if box.checked
    Engine.sticky = true
  else
    Engine.sticky = false
