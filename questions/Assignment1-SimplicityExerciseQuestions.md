Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: 5.64 mA
   A1_Q1.png
   


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer:5.74 mA
   A1_Q2.png



**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**

   Answer: The difference between the Instantaneous currents of Q1 and Q2 might be ~1% but that is insignificant. And the difference between the average currents is ~0%. The drive strengths of the GPIO pins is meant to indicate how much maximum current a given pin can draw or sink while maintaining the necessary minimum high and maximum low output voltages. This does not limit the current being drawn or sunk through that pin.
   In other words, it can determine the max load a pin can drive without losing quality of the output signal. In the given schematic, LED0  and LED1 has a series resistance of 3k Ohm and being powered by a 3.3V supply. 
   In our case, a weak drive strength means drawing more current than 1mA might not damage the board but does not guarantee output voltage levels.
   Similarly, drawing 10mA in strong mode still guarantees specified output voltage levels.
   


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: 5.19 mA
   A1_Q4.png



**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer:5.36 mA
   A1_Q5.png



