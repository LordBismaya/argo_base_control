README
======

This is the base controller node for the ARGO. It subscribes to the Joy node and publishes cmd_vel under the topic /argo_base/cmd_vel

Update (25/10/2016):
======
1. Brake ADC values checked. The Servo Arduino Reads the brake ADC values. ADC0 reads Right Brake and ADC1 reads Left Brake 
2. The range of values are LEFT(140-270) and RIGHT(130-300). 
3. The right one is a bit less contrained which allowes it to expand a bit more. Assumming it will be tightened soon, we will stick to 140 and 270 as a uniform range.
