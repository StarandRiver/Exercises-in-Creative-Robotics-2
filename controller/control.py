import time
import board
import digitalio
import pwmio
import usb_hid
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode

#�L�[�{�[�h�f�o�C�X�̃Z�b�g�A�b�v
kbd = Keyboard(usb_hid.devices)

#�e�X�C�b�`�̒[�q�ݒ�
sw1 = digitalio.DigitalInOut(board.GP1)	#�{�^��1(�g���N�I�tLEFT_BRACKETfRIGHT_BRACKET)
sw1.direction = digitalio.Direction.INPUT
sw1.pull = digitalio.Pull.UP

sw2 = digitalio.DigitalInOut(board.GP3)	#�{�^��2(�W���p��LEFT_BRACKETiRIGHT_BRACKET)
sw2.direction = digitalio.Direction.INPUT
sw2.pull = digitalio.Pull.UP

sw3 = digitalio.DigitalInOut(board.GP2)	#�{�^��3(�g���N�I��)
sw3.direction = digitalio.Direction.INPUT
sw3.pull = digitalio.Pull.UP

sw4 = digitalio.DigitalInOut(board.GP13)	#�O�g(LEFT_BRACKETwRIGHT_BRACKET)
sw4.direction = digitalio.Direction.INPUT
sw4.pull = digitalio.Pull.UP

sw5 = digitalio.DigitalInOut(board.GP12)	#�E����(LEFT_BRACKETdRIGHT_BRACKET)
sw5.direction = digitalio.Direction.INPUT
sw5.pull = digitalio.Pull.UP

sw6 = digitalio.DigitalInOut(board.GP11)	#���(LEFT_BRACKETsRIGHT_BRACKET)
sw6.direction = digitalio.Direction.INPUT
sw6.pull = digitalio.Pull.UP

sw7 = digitalio.DigitalInOut(board.GP7)		#������(LEFT_BRACKETaRIGHT_BRACKET)
sw7.direction = digitalio.Direction.INPUT
sw7.pull = digitalio.Pull.UP


#LED�̒[�q�ݒ�
led = pwmio.PWMOut(board.GP0, frequency=5000)
Powerled = pwmio.PWMOut(board.GP26, frequency=5000)

#LED�_��
Powerled.duty_cycle = 8000
led.duty_cycle = 8000

#���O�̃X�C�b�`���
old_sw1_value = True
old_sw2_value = True
old_sw3_value = True
old_sw4_value = True
old_sw5_value = True
old_sw6_value = True
old_sw7_value = True

#���C�����[�v
while True:
    #�u�U�[��炷�t���O������

    #�e�X�C�b�`�̉����������m
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

    #���O�̃X�C�b�`��Ԃ̍X�V
    old_sw1_value = sw1.value
    old_sw2_value = sw2.value
    old_sw3_value = sw3.value
    old_sw4_value = sw4.value
    old_sw5_value = sw5.value
    old_sw6_value = sw6.value
    old_sw7_value = sw7.value

    #�X�C�b�`���͑҂�
    time.sleep(0.01)