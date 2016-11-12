{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// METABOT3 Motor Controller
//
// Serial interface:
//
// set Motor speed : +ss <motor:0..3> <speed:0..255> <dir:0|1>
// get Motor position counts: +gp
// get Motor speeds: +gs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}

CON
  _CLKMODE = xtal1 + pll16x
  _XINFREQ = 6_000_000

OBJ
  ps : "propshell"
  quad :  "encoder"
  pwm : "pibal_pwm"

DAT
  encoderPins byte 11, 12, 13, 14, 15, 16, 17, 18
  motorEn     byte 0, 5, 19, 24
  motorD1     byte 1, 4, 21, 22
  motorD2     byte 2, 3, 20, 23

VAR
  long  stack[64]
  long  lastpos[4]
  long  speed[4]
  
PUB main
  ps.init(string(">"), string("?"), 115200, 31, 30)    ' start the command interpreter shell
  quad.Start(@encoderPins)                       ' start the quadrature encoder reader (1 x cog)
  resetMotors                                    ' reset the motors
  pwm.start_pwm(motorEn[0], motorEn[1], motorEn[2], motorEn[3], 16000)    ' start the pwm driver (2 x cogs)
  cognew(pid, @stack)

  repeat
    result := ps.prompt
    \cmdHandler(result)

PRI cmdHandler(cmdLine)
  cmdSetSpeed(ps.commandDef(string("+ss"), string("Set speed <motor[0..3], speed[-1000...1000]>") , cmdLine))
  cmdSetForward(ps.commandDef(string("+sf"), string("Set forward <motor[0..3], dir[0, 1]>") , cmdLine))
  cmdSetStop(ps.commandDef(string("+s"), string("Stop") , cmdLine))
  cmdGetPos(ps.commandDef(string("+gp"), string("Get position") , cmdLine))
  cmdGetSpeed(ps.commandDef(string("+gs"), string("Get speed") , cmdLine))
  return true
  
PRI cmdSetSpeed(forMe) | motor, newspeed, dir
  if not forMe
    return
  ps.parseAndCheck(1, string("!ERR 1"), true)
  motor := ps.currentParDec
  if motor < 0 or motor > 3
    ps.puts(string("!ERR 2", ps#CR))
    abort
  ps.parseAndCheck(2, string("!ERR 3"), true)
  newspeed := ps.currentParDec
  ps.puts(string("Set Motor Speed "))
  ps.putd(motor)
  ps.puts(string(", "))
  ps.putd(newspeed)
  ps.puts(string(ps#CR))

  pwm.set_duty(motor, newspeed)
  setMotorDir(motor, newspeed)
  ps.commandHandled

PRI cmdSetForward(forMe) | motor, dir
  if not forMe
    return
  ps.parseAndCheck(1, string("!ERR 1"), true)
  motor := ps.currentParDec
  if motor < 0 or motor > 3
    ps.puts(string("!ERR 2", ps#CR))
    abort
  ps.parseAndCheck(2, string("!ERR 3"), true)
  dir := ps.currentParDec
  
  ps.puts(string("Set Motor Full Speed "))
  ps.putd(motor)
  ps.puts(string(", "))
  ps.putd(dir)
  ps.puts(string(ps#CR))
  resetMotors
  if dir == 0
    outa[motorEn[motor]] := %0
  else
    outa[motorEn[motor]] := %1
  setMotorDir(motor, dir)
  ps.commandHandled
  
PRI cmdSetStop(forMe)
  if not forMe
    return
  resetMotors
  ps.puts(string("Stopped"))
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetPos(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(lastpos[i])
    ps.puts(string(" "))
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetSpeed(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(speed[i])
    ps.puts(string(" "))
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI resetMotors | i
  repeat i from 0 to 3
    outa[motorEn[i]] := %0
    outa[motorD1[i]] := %0
    outa[motorD2[i]] := %0
    dira[motorEn[i]] := %1
    dira[motorD1[i]] := %1
    dira[motorD2[i]] := %1

PRI setMotorDir(motor, dir)
  if dir == 0
    outa[motorD1[motor]] := %0
    outa[motorD2[motor]] := %0
  elseif dir > 0
    outa[motorD1[motor]] := %0
    outa[motorD2[motor]] := %1
  else
    outa[motorD1[motor]] := %1
    outa[motorD2[motor]] := %0

PRI pid | i, nextpos
  nextpos := 0
  repeat
    waitcnt(clkfreq / 100 + cnt)
    repeat i from 0 to 3
      nextpos := quad.count(i)
      speed[i] := nextpos - lastpos[i]
      lastpos[i] := nextpos

  
  
  