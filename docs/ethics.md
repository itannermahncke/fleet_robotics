# Ethics

In the world of robotics, taking ethics into consideration is quite vital. Without roboethical conversations, our creations as engineers can be used for purposes outside of what was initially designed. The last thing we want is to let the military make a fleet of Neatos storm some innocent town.

So, decided to examine the ethical implications of our project. We analyzed the ethics of our project according to the [CVPR framework](https://cvpr2022.thecvf.com/ethics-guidelines), summarized below

Ethical Concerns
-

1. **Facilitates Harm:** Ensure the technology cannot be directly used in weapons or systems that could cause physical harm.  

2. **Safety and Privacy Risks:** Assess whether the application could cause accidents, create security vulnerabilities, or expose personal information without consent.  

3. **Human Rights Concerns:** Avoid designs that could discriminate, restrict access to vital services, or negatively impact fundamental rights like healthcare or employment.  

4. **Economic and Workplace Impact:** Evaluate whether the technology undermines workers' dignity, autonomy, or safety, or increases invasive surveillance.  

5. **Surveillance Risks:** Avoid creating tools that could enable bulk surveillance, criminal profiling, or predictive analyses of protected characteristics.  

6. **Environmental Harm:** Consider whether the technology contributes to environmental degradation, like deforestation or pollution.  

7. **Deceptive Practices:** Prevent uses that could mislead people, facilitate fraud, or impersonate others to cause harm or disrupt social processes.  

## We looked at these ethical concerns, and summarized how our project does or doesn't follow these below:

1. **Facilitates Harm:** We don't believe that the Neatos can directly be involved in physical harm. However, we are not oblivious to the fact that general fleet robotics can be adapted to weapons systems. 

2. **Safety and Privacy Risks:** We dont believe that our project can pose any risks to accidents, security issues, or fundamental rights. 

3. **Human Rights Concerns:** We don't believe that our project impacts anyone's human rights. 

4. **Economic and Workplace Impact:** While it would be pretty cool to have a fleet of Neatos take the jobs of workers... its pretty clear that our little fleet has no chance of taking any jobs. 

5. **Surveillance Risks:** Since our camera data is only saved for self odometry purposes, it can not be used in any kind of survailence.

6. **Environmental Harm:** Our project does not directly have any effect on the environment (other than the materials used to create the Neatos)

7. **Deceptive Practices:** There is no form of deception used in our project, and we prove this by publicly hosting our entire code repo on github

## Model Card
By following this [Model Card Framework](https://arxiv.org/pdf/1810.03993), we created a card to make clear the intentions and implications of our robotic system:

1. **Algorithm Details:**  
   - Ivy Mahncke, Ariel Chen, Vivian Mak, Charlie Mawn
   - December 19, 2024
   - Fleet robotics path planner using sensor fusion and visual odometry
   - For questions or concerns, message any collaborators on [our github](https://github.com/itannermahncke/fleet_robotics)  

2. **Intended Use:**  
   - Our primary intended uses are simply for learning. While our goal is to create this project that does multi-robot path planning, our true motivation is for learning. 
   - Our primary intended users include us, the creaters, as well as anyone who wants to expand their robotics algorithm understand and has access to the resources we have (so pretty specific to Olin College)

3. **Factors:**  
   - Our only real factors that can effect performance are environmental. We require an indoor space, with enough room to draw a defined map. 

4. **Metrics:**  
   - Due to the scope of our project, we lack any true metrics of how efficient our algorithms are.  

5. **Ethical Considerations:**  
   - Reference our ethical statement above to learn more about how we considered ethics in our design.  

6. **Caveats and Recommendations:**  
   - Our biggest known issue is that our algorithm can not fully be stopped without completely restarting the machine.