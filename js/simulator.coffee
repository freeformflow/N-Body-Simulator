# This is the "naive" version of the 2D N-Body simulator.

# Initialize the HTML Canvas viewport.
canvas = 0
ctx = 0
overlay = 0
backgroundColor = '#ffffff'
bodyColor =  "#000" #'#0193de'
bodySize = 0
bodyBoundary = 0
yReference = 0
renderID = 0

# Declare the variables that track cursor position within the display.
userX = userY = 0

# Declare the simulation memory: position, velocity, acceleration
posX = posY = velX = velY = accX = accY = 0

# Initialize constants.
twoPi = 2 * Math.PI


# Create an object to gather together simulation parameters.  These control the
# performance and physics within the simulation.  All of them can be manipulated
# by the user via the user interface.
window.Engine = {}


# Seed the random number generator that allows us to scatter particles evenly at
# the simulation start.
randomSeed = new Date().getTime()


window.initializeSimulation = ->
  Engine.run = true
  Engine.n = Number document.getElementById("n_control").value
  Engine.G = Number document.getElementById("G_control").value
  if window.innerWidth < 768
    Engine.dt = Number document.getElementById("dt_control").value / 2
    document.getElementById("dt_control").value = Engine.dt
  else
    Engine.dt = Number document.getElementById("dt_control").value

  Engine.initial_speed = Number document.getElementById('Vi_control').value
  Engine.wall = Number document.getElementById("wall_control").value
  Engine.user_input = document.getElementById('input_control').checked
  #Engine.collide = document.getElementById('collide_control').checked
  Engine.collid = Engine.sticky = document.getElementById('sticky_control').checked


  canvas = document.getElementById "view"
  ctx = canvas.getContext "2d"
  overlay = document.getElementById "canvasOverlay"
  window.sizeCanvas()
  seedSimulation()
  update()






#-------------------------------------------------------------------------------
# This block contains code for rendering the simluation on-screen.
#-------------------------------------------------------------------------------

# Draws the current state of the simulation as a batch.  We use the somewhat
# awkward decrementing while loop to leverage V8 optimizations.
renderFrame = () ->
  ctx.fillStyle = bodyColor
  ctx.beginPath()

  {length} = posX
  i = 0
  while length--
    x = posX[i]
    y = posY[i]
    ctx.moveTo x, y
    ctx.arc x, y, bodySize, 0, twoPi, true  # Draws a circle.
    i++

  ctx.fill()



# Size the canvas according how the user is usering the page.
window.sizeCanvas = () ->
  w = window.innerWidth
  h = window.innerHeight - document.getElementById("controlPanel").offsetHeight

  canvas.width = w
  canvas.height = h
  canvas.style.width = "#{w}px"
  canvas.style.height = "#{h}px"
  overlay.style.maxHeight = "#{h}px"
  document.getElementById("settingsOverlay").style.height = "#{h}px"

  bodySize = if w > h then (h / 250) else (w / 200)
  bodyBoundary = 4 * bodySize * bodySize

  # We'll use this value to calculate the cursor's position within the canvas.
  yReference = document.getElementById("controlPanel").offsetHeight




#-------------------------------------------------------------------------------
# This block contains code for driving the simulation's integration.
#-------------------------------------------------------------------------------

randomNumber = () ->
  # This is a random number generator.  Here, we implement it
  # by linking two linear congruential generators.  This function
  # returns a number between 0 and 1, inclusive.

  A =  (1664525 * randomSeed + 1013904223) % 4294967296
  randomSeed =  (1103515245 * A + 12345) % 2147483648

  return randomSeed / 2147483648

seedSimulation = () ->
  # This function seeds the simulation with particles by setting their initial
  # position through the use of a random number generator. We also apply a
  # non-pseudorandom velocity field on the particles to set them spining around
  # the center of the display as a starting place.

  # Reset the simulation memory arrays.
  posX = []
  posY = []
  velX = []
  velY = []
  accX = []
  accY = []

  smallSample = false

  if smallSample
    Engine.n = 4
    posX = [100, 200, 300, 400]
    posY = [200, 200, 200, 200]
    velX = [0, 0, 0, 0]
    velY = [0, 0, 0, 0]
    accX = [0, 0, 0, 0]
    accY = [0, 0, 0, 0]
  else

    speed = Engine.initial_speed
    for i in [0...Engine.n]
      x = randomNumber() * canvas.width
      y = randomNumber() * canvas.height

      posX.push x
      posY.push y

      # Apply a velocity tangential to the line connecting the point to the
      # display's center, but with speed of zero for those close to center.
      x -= canvas.width / 2
      y -= canvas.height / 2
      dist = Math.sqrt(x*x + y*y)

      if dist > 10
        velX.push       speed * (y / dist)
        velY.push  -1 * speed * (x / dist)
      else
        velX.push 0
        velY.push 0

      # Apply no acceleration.
      accX.push 0
      accY.push 0



trackCursor = (event) ->
  userX = event.clientX
  userY = event.clientY - yReference


forceEvaluation = () ->
  # This function performs a force evaluation for the system.  The basic
  # algorithm is to take a single particle and determine the sum of the force
  # exerted by all other particles. Then repeat the process for all particles.
  # This is called every time a new frame is rendered, so it's important to
  # use any trick we can to avoid unneccessary work.  For looping, that means
  # using the decrementing while trick.

  # Zero out every particle's acceleration.
  i = 0
  l = Engine.n
  while l--
    accX[i] = 0
    accY[i] = 0
    i++

  # These arrays keep track of particle data if they are undergoing a collision.
  if Engine.collide
    collide = true
    collide_i = []
    collide_j = []
    collide_x = []
    collide_y = []
    collide_idist = []

  # We can more efficently calculate this pair-wise sum by doing it for only the
  # Upper Triangle of the pair matrix.  This way we only calculate the distance
  # between each pair only once.
  G = Engine.G
  cutoff = 100
  inverseCutoff = 1 / cutoff

  l1 = Engine.n - 1
  i = 1
  while l1--
    l2 = i
    j = 0
    while l2--

      # Start by geting the distance between i and j
      x = posX[i] - posX[j]
      y = posY[i] - posY[j]
      dist_sq = x*x + y*y

      # Use this distance to get the force between i and j
      # Force = -(dE/dr), which for gravitation is (G * m_i * m_j) / distance**2
      # We can ignore a lot those terms since we are working with reduced units
      # and are cheating to get better visuals.

      # Only apply this to particles that are far enough apart. Otherwise, the
      # particles sling themselves past one another. (Fixed timestep size is too
      # big to handle perige properly.)
      if dist_sq > cutoff
        force = G / dist_sq
        dist_inverse = 1 / Math.sqrt(dist_sq)
      else
        if collide and dist_sq < bodyBoundary
          # If the user has requested particle-particle collisions, we note any overlapping
          # particles here.  We address these pairs inside applyConstraints()
          collide_i.push i
          collide_j.push j
          collide_x.push x
          collide_y.push y
          collide_idist.push 1 / Math.sqrt(dist_sq)

        # After recording the values, fudge force to avoid integration errors.
        force = inverseCutoff * G
        dist_inverse = inverseCutoff


      Fx = force * x * dist_inverse
      Fy = force * y * dist_inverse

      # This force is applied to i.
      accX[i] = accX[i] - Fx
      accY[i] = accY[i] - Fy

      # Now the opposite and equal force is applied to j.
      accX[j] = accX[j] + Fx
      accY[j] = accY[j] + Fy
      j++
    i++

  Engine.collide_i = collide_i
  Engine.collide_j = collide_j
  Engine.collide_x = collide_x
  Engine.collide_y = collide_y
  Engine.collide_idist = collide_idist



  # Now, if the user has opted to use the cursor as an attractor of particles,
  # we calculate those forces here.
  if Engine.user_input && userX != null
    l = Engine.n
    i = 0
    while l--
      x = posX[i] - userX
      y = posY[i] - userY
      dist_sq = x*x + y*y
      dist_inverse = 1 / Math.sqrt(dist_sq)

      if dist_sq > 400
        force = 10000 / dist_sq

        accX[i] = accX[i] - force * x * dist_inverse
        accY[i] = accY[i] - force * y * dist_inverse
      i++



integrateDynamics = () ->
  # Dynamics are achieved using Velocity-Verlet Integration.
  # Step 1: pos(t + dt)       = pos(t) + vel(t) * dt + 0.5 * acc(t) * dt**2
  # Step 2: vel(t + 0.5 * dt) = vel(t) + 0.5 * acc(t) * dt
  # Step 3: acc(t + dt)       = Force Evaluation / mass  @time = t + dt
  # Step 4: vel(t + dt)       = vel(t + 0.5 * dt) + 0.5 * acc(t + dt) * dt

  dt = Engine.dt
  dt2 = 0.5 * dt * dt
  half_dt = 0.5 * dt

  # Step 1:
  l = Engine.n
  i = 0
  while l--
    posX[i] += velX[i] * dt + accX[i] * dt2
    posY[i] += velY[i] * dt + accY[i] * dt2
    i++

  # Step 2:
  l = Engine.n
  i = 0
  while l--
    velX[i] += accX[i] * half_dt
    velY[i] += accY[i] * half_dt
    i++

  # Step 3:
  forceEvaluation()

  # Step 4:
  l = Engine.n
  i = 0
  while l--
    velX[i] += accX[i] * half_dt
    velY[i] += accY[i] * half_dt
    i++




applyConstraints = () ->
  # Apply constraints.  These will keep the particles within the box, because if
  # they get loose, there is not enough force to bring them back.  We can also use the
  # walls to bleed energy out of the system and make it behave better.  If a
  # particle goes past a wall, it is put back within bounds and its velocity
  # in that direction may be modified.

  #=====================================================
  # Collisions
  #=====================================================

  if Engine.collide and Engine.collide_i.length > 0
    # If the user has requested particle-particle collisions, we calculate here.
    # We model these collisions as inelastic, meaning only momentum is conserved.

    separation = 2 * bodySize + 2
    {abs} = Math

    collide_i = Engine.collide_i
    collide_j = Engine.collide_j
    collide_x = Engine.collide_x
    collide_y = Engine.collide_y
    collide_idist = Engine.collide_idist

    if Engine.sticky
      separation *= 0.5
      l = collide_i.length
      k = 0
      while l--
        i = collide_i[k]
        j = collide_j[k]
        x = collide_x[k]
        y = collide_y[k]
        dist_inverse = collide_idist[k]

        # Move the particles so they partially overlap.
        posX[i] = posX[j] + separation * x * dist_inverse
        posY[i] = posY[j] + separation * y * dist_inverse

        # Modify both particle's velocity.  Because they have the same mass
        # their final velcities are equal and the average of their inital velocities.
        # m_i * v_i  + m_j * v_j  = M * V   =>  V = (v_i + v_j) / 2
        velX[i] = velX[j] = 0.5 * (velX[i] + velX[j])
        velY[i] = velY[j] = 0.5 * (velY[i] + velY[j])
        k++

    else

      l = collide_i.length
      k = 0
      while l--
        i = collide_i[k]
        j = collide_j[k]
        x = collide_x[k]
        y = collide_y[k]
        dist_inverse = collide_idist[k]

        # Move the particles so they are no longer overlapping.
        posX[i] = posX[j] + separation * x * dist_inverse
        posY[i] = posY[j] + separation * y * dist_inverse

        # Maybe modify both particle's velocity.
        vel_ij_x = velX[i] - velX[j]
        vel_ij_y = velY[i] - velY[j]

        if vel_ij_x > 0 and vel_ij_y > 0
          # The particles are moving apart.  Don't do anything to them.
          continue
        else if vel_ij_x > vel_ij_y and abs(vel_ij_x) > abs(vel_ij_y)
          # The particles are moving apart.  Don't do anything to them.
          continue
        else if vel_ij_x < vel_ij_y and abs(vel_ij_x) < abs(vel_ij_y)
          # The particles are moving apart.  Don't do anything to them.
          continue

        velX[i] = -velX[i]
        velY[i] = -velY[i]
        velX[j] = -velX[j]
        velY[j] = -velY[j]

        k++





  #=====================================================
  # Box Boundaries
  #=====================================================

  min = bodySize
  max_width = canvas.width - bodySize
  max_height = canvas.height - bodySize
  wall = Engine.wall

  l = Engine.n
  i = 0
  while l--

    if posX[i] < min
      # particle has gone off the left edge.
      posX[i] = min
      velX[i] = -wall * velX[i]
      accX[i] = 0

    else if posX[i] > max_width
      # particle has gone off the right edge.
      posX[i] = max_width
      velX[i] = -wall * velX[i]
      accX[i] = 0

    else if posY[i] < min
      # particle has gone off the top edge.
      posY[i] = min
      velY[i] = -wall * velY[i]
      accY[i] = 0

    else if posY[i] > max_height
      # particle has gone off the bottom edge.
      posY[i] = max_height
      velY[i] = -wall * velY[i]
      accY[i] = 0

    i++


  #=====================================================
  # Velocity Governor
  #=====================================================
  speed = 500
  l = Engine.n
  i = 0
  while l--
    if velX[i] < -speed
      velX[i] = -speed
    else if velX[i] > speed
      velX[i] = speed

    if velY[i] < -speed
      velY[i] = -speed
    else if velY[i] > speed
      velY[i] = speed






#=============================
# Main
#=============================
# This gets called repeatedly to iterate the simulation and draw the results.
update = () ->
  # We need to carefully time the animation loops.  We start our stopwatch now and
  # check it when we're done drawing.
  start = new Date().getTime()

  # Begin drawing.... First, wipe the view.
  ctx.clearRect 0, 0, canvas.width, canvas.height

  # Now, create a new background.
  ctx.fillStyle = backgroundColor
  ctx.fillRect 0, 0, canvas.width, canvas.height

  # Update the dynamics using the simulator's Engine
  integrateDynamics()
  applyConstraints()

  # Finally, draw all the particles.
  renderFrame()

  # We're done drawing.  Check the stopwatch now.
  duration = new Date().getTime() - start

  # Rinse and repeat...  This will recurse indefinitely.
  # We need at least 25 frames per second to give realistic motion.
  # That is 40 ms per frame.  If we're lucky, we have time to spare, and
  # we can ask the CPU to idle.  Otherwise, move on to the next frame right away,
  # but lag will be noticeable...  :(
  if duration < 17
    renderID = window.setTimeout (-> update()), 17 - duration
  else
    renderID = window.setTimeout (-> update()), 0



#-------------------------------------------------------------------
# This code block handles user controls and updates settings here.
#-------------------------------------------------------------------

window.togglePlay = ->
  play = document.getElementsByClassName("glyphicon-play")[0]
  pause = document.getElementsByClassName("glyphicon-pause")[0]

  if Engine.run
    play.classList.remove "hidden"
    pause.classList.add "hidden"
    window.clearTimeout renderID
    Engine.run = false
  else
    play.classList.add "hidden"
    pause.classList.remove "hidden"
    Engine.run = true
    update()

window.restart = ->
  play = document.getElementsByClassName("glyphicon-play")[0]
  pause = document.getElementsByClassName("glyphicon-pause")[0]

  if "hidden" in pause.classList
    play.classList.add "hidden"
    pause.classList.remove "hidden"
  Engine.run = true

  window.clearTimeout renderID
  seedSimulation()
  update()


window.toggleOverlay = ->
  settings = document.getElementById "settingsOverlay"
  if "hidden" in settings.classList
    settings.classList.remove "hidden"
  else
    settings.classList.add "hidden"


window.onresize = ->
  window.sizeCanvas()




window.setN = () ->
  old = Engine.n
  Engine.n = Number document.getElementById("n_control").value
  window.restart()

window.setG = () ->
  Engine.G = Number document.getElementById("G_control").value

window.setDt = () ->
  Engine.dt = Number document.getElementById("dt_control").value

window.setVi = () ->
  Engine.initial_speed = Number document.getElementById("Vi_control").value

window.setWall = () ->
  Engine.wall = Number document.getElementById('wall_control').value

window.setInput = () ->
  Engine.user_input = document.getElementById('input_control').checked

  if Engine.user_input
    canvas.addEventListener "mousemove", (e) -> trackCursor(e)
  else
    canvas.removeEventListener "mousemove"

# window.setCollide = () ->
#   Engine.collide = document.getElementById('collide_control').checked

window.setSticky = () ->
  Engine.collide = Engine.sticky = document.getElementById('sticky_control').checked

  if Engine.sticky
    bodyBoundary /= 3
  else
    bodyBoundary *= 3
