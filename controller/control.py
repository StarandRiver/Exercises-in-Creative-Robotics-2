import time
import board
import digitalio
import pwmio
import usb_hid
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode

#キーボードデバイスのセットアップ
kbd = Keyboard(usb_hid.devices)

#各スイッチの端子設定
sw1 = digitalio.DigitalInOut(board.GP1)	#ボタン1(トルクオフLEFT_BRACKETfRIGHT_BRACKET)
sw1.direction = digitalio.Direction.INPUT
sw1.pull = digitalio.Pull.UP

sw2 = digitalio.DigitalInOut(board.GP3)	#ボタン2(標準姿勢LEFT_BRACKETiRIGHT_BRACKET)
sw2.direction = digitalio.Direction.INPUT
sw2.pull = digitalio.Pull.UP

sw3 = digitalio.DigitalInOut(board.GP2)	#ボタン3(トルクオン)
sw3.direction = digitalio.Direction.INPUT
sw3.pull = digitalio.Pull.UP

sw4 = digitalio.DigitalInOut(board.GP13)	#前身(LEFT_BRACKETwRIGHT_BRACKET)
sw4.direction = digitalio.Direction.INPUT
sw4.pull = digitalio.Pull.UP

sw5 = digitalio.DigitalInOut(board.GP12)	#右旋回(LEFT_BRACKETdRIGHT_BRACKET)
sw5.direction = digitalio.Direction.INPUT
sw5.pull = digitalio.Pull.UP

sw6 = digitalio.DigitalInOut(board.GP11)	#後退(LEFT_BRACKETsRIGHT_BRACKET)
sw6.direction = digitalio.Direction.INPUT
sw6.pull = digitalio.Pull.UP

sw7 = digitalio.DigitalInOut(board.GP7)		#左旋回(LEFT_BRACKETaRIGHT_BRACKET)
sw7.direction = digitalio.Direction.INPUT
sw7.pull = digitalio.Pull.UP


#LEDの端子設定
led = pwmio.PWMOut(board.GP0, frequency=5000)
Powerled = pwmio.PWMOut(board.GP26, frequency=5000)

#LED点灯
Powerled.duty_cycle = 8000
led.duty_cycle = 8000

#直前のスイッチ状態
old_sw1_value = True
old_sw2_value = True
old_sw3_value = True
old_sw4_value = True
old_sw5_value = True
old_sw6_value = True
old_sw7_value = True

#メインループ
while True:
    #ブザーを鳴らすフラグ初期化

    #各スイッチの押下時を検知
    if sw1.value == False and old_sw1_value == True:
	    led.duty_cycle = 0
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.f)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    if sw2.value == False and old_sw2_value == True:
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.i)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    if sw3.value == False and old_sw3_value == True:
	    led.duty_cycle = 8000
	    kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.n)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)
        
    if sw4.value == False and old_sw4_value == True:
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.w)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    if sw5.value == False and old_sw5_value == True:
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.d)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    if sw6.value == False and old_sw6_value == True:
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.s)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    if sw7.value == False and old_sw7_value == True:
        kbd.send(Keycode.LEFT_BRACKET)
	    kbd.send(Keycode.a)
	    kbd.send(Keycode.RIGHT_BRACKET)
	    kbd.send(Keycode.RETURN)

    #直前のスイッチ状態の更新
    old_sw1_value = sw1.value
    old_sw2_value = sw2.value
    old_sw3_value = sw3.value
    old_sw4_value = sw4.value
    old_sw5_value = sw5.value
    old_sw6_value = sw6.value
    old_sw7_value = sw7.value

    #スイッチ入力待ち
    time.sleep(0.01)