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
  debuglim = 8

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
  long  debug[debuglim]
  long  desired_speed[4]
  long  actual_speed[4]
  long  error_integral[4]
  long  error_derivative[4]
  long  millidiv
  long  millioffset
  long Kp
  long Ki
  long Kd
  
PUB main
  millidiv := clkfreq / 1000
  Kp := 20
  Ki := 2
  Kd := 10
  millioffset := negx / millidiv * -1
  ps.init(string(">"), string("?"), 115200, 31, 30)    ' start the command interpreter shell
  quad.Start(@encoderPins)                       ' start the quadrature encoder reader (1 x cog)
  resetMotors                                    ' reset the motors
  pwm.start_pwm(motorEn[0], motorEn[1], motorEn[2], motorEn[3], 16000)    ' start the pwm driver (2 x cogs)
  cognew(pid, @stack)
  ' ps.puts(string("clkfreq : "))
  ' ps.putd(clkfreq)
  ' ps.puts(ps#CR)

  repeat
    result := ps.prompt
    \cmdHandler(result)

PRI cmdHandler(cmdLine)
  cmdSetSpeed(ps.commandDef(string("+ss"), string("Set speed <motor[0..3], speed[-100...100]>") , cmdLine))
  cmdSetSpeedAll(ps.commandDef(string("+sa"), string("Set all speeds <speed[-100...100] x 4>") , cmdLine))
  cmdSetStop(ps.commandDef(string("+st"), string("Stop") , cmdLine))
  cmdSetPID(ps.commandDef(string("+sp"), string("Set PID Parameters <Kp Ki Kd>") , cmdLine))
  cmdGetPos(ps.commandDef(string("+gp"), string("Get position") , cmdLine))
  cmdGetSpeed(ps.commandDef(string("+gs"), string("Get speed") , cmdLine))
  cmdGetTime(ps.commandDef(string("+gt"), string("Get time") , cmdLine))
  cmdGetDebug(ps.commandDef(string("+gd"), string("Get debug") , cmdLine))
  ps.puts(string("? for help", ps#CR))  ' no command recognised
  return true

PRI cmdSetSpeedAll(forMe) | motor, newspeed
  if not forMe
    return
  ps.puts(string("Set Motor Speed "))
  repeat motor from 0 to 3
    ps.parseAndCheck(motor+1, string("!ERR 1"), true)
    newspeed := ps.currentParDec
    ps.putd(newspeed)
    ps.puts(string(" "))
    desired_speed[motor] := newspeed
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdSetSpeed(forMe) | motor, newspeed
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
  desired_speed[motor] := newspeed
  ps.commandHandled

PRI cmdSetStop(forMe)
  if not forMe
    return
  resetMotors
  ps.puts(string("Stopped"))
  ps.puts(string(ps#CR))
  ps.commandHandled

PRI cmdSetPID(forMe) | motor, newKp, newKi, newKd
  if not forMe
    return
  ps.parseAndCheck(1, string("!ERR 1"), true)
  newKp := ps.currentParDec
  ps.parseAndCheck(2, string("!ERR 2"), true)
  newKi := ps.currentParDec
  ps.parseAndCheck(3, string("!ERR 3"), true)
  newKd := ps.currentParDec

  ps.puts(string("Set PID Parameters "))
  ps.putd(newKp)
  ps.puts(string(", "))
  ps.putd(newKi)
  ps.puts(string(", "))
  ps.putd(newKd)
  ps.puts(string(ps#CR))
  Kp := newKp
  Ki := newKi
  Kd := newKd
  ps.commandHandled
  
PRI cmdGetPos(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(lastpos[i])
    ps.puts(string(" "))
  ps.putd(millioffset + cnt/millidiv)  
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetSpeed(forMe) | i
  if not forMe
    return
  repeat i from 0 to 3
    ps.putd(actual_speed[i])
    ps.puts(string(" "))
  ps.putd(millioffset + cnt/millidiv)
  ps.puts(string(ps#CR))
  ps.commandHandled

PRI cmdGetTime(forMe)
  if not forMe
    return
  ps.putd(millioffset + cnt/millidiv)
  ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI cmdGetDebug(forMe) | i
  if not forMe
    return
  ps.puts(string(ps#CR))
  repeat i from 0 to debuglim-1
    ps.putd(i)
    ps.puts(string(": "))
    ps.putd(debug[i])
    ps.puts(string(ps#CR))
  ps.commandHandled
  
PRI resetMotors | i
  repeat i from 0 to 3
    desired_speed[i] := 0
    outa[motorEn[i]] := %0
    outa[motorD1[i]] := %0
    outa[motorD2[i]] := %0
    dira[motorEn[i]] := %1
    dira[motorD1[i]] := %1
    dira[motorD2[i]] := %1

PRI pid | i, nextpos, error, last_error, nexttime, newspeed
  nextpos := 0
  nexttime := millidiv + cnt
  resetMotors    ' enables the direction ports control from this cog
  repeat
    waitcnt(nexttime)
    nexttime += millidiv * 2
    'Here every two milliseconds
   
    repeat i from 0 to 3                ' loop takes just under 1ms to complete
      debug[i] := desired_speed[i]
      nextpos := quad.count(i)
      last_error := desired_speed[i] - actual_speed[i] 
      actual_speed[i] := nextpos - lastpos[i]
      lastpos[i] := nextpos

      error := desired_speed[i] - actual_speed[i] 
      error_derivative[i] := error - last_error
      error_integral[i] += error
      newspeed := Kp * error + Ki * error_integral[i] + Kd * error_derivative[i]
      setMotorSpeed(i, newspeed)
      debug[i+4] := pwm.get_duty(i)

PRI setMotorSpeed(motor, speed)
  pwm.set_duty(motor, speed)
  if speed == 0
    outa[motorD1[motor]] := %0
    outa[motorD2[motor]] := %0
  elseif speed > 0
    outa[motorD1[motor]] := %0
    outa[motorD2[motor]] := %1
  else
    outa[motorD1[motor]] := %1
    outa[motorD2[motor]] := %0
  
  
  