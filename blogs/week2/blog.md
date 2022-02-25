# Week 2 blog
**Date:** 7 Feb - 13 Feb

In a meeting with aa on Monday, aa set us a new task to make a gantt chart. The gantt chart makes the structure of our project clearer, and also allows members to collaborate with each other and allocate time more easily. Also our main task this week is to select the model we want to simulate and import the quadruped robot model file into pybullet. Learn how to use pybullet to simulate manipulating a robot. After the previous work is done, we plan everyone to try to write some simple code to make the robot move. For example, manipulating a joint of a robot and bending it.


## Goals for week 2

1. Select the quadruped robot model to use.
2. Import the model into pybullet.
3. Initially configure the simulation environment. e.g. gravity, simulated scenes, etc.
4. Write simple code to move a joint and test.

## Implementation of the task

![model](https://lh3.googleusercontent.com/U8GUx2sgsVieIICqEgiazMsGqXptXL9tmfXpzn8VztxdqWD4ricglgwzJwiRsnmyEP64UYMQOjr9lIF-WkC8dvXexdbe0wlovrKcpAPSQzrcMQ8h2LEF1rHCLQZw4PU3zXcTJ-S9AWLt1iO-0idYR32qo2MjG3D7nOGyhdhnwiHQ7bGRV9XxJFVv89oN1ksSqtGxl5FhdeDEutfsBKFWWSqgUbUMl6i_urTQzV_Fq4FhgQgtD4fLbyylNMSnzXVogn7AR1xOvY2PYc_Ib_qenH9PE2fy3jE-Tyup80JpGVoyXxDYGdcU6qoEw_Ie8zBVFwuE7ZkRlxCN3LH3RwP2FZO0xFPvQcLL3Lkldil6Bx87TLXEqDKtwuEe4X_J4mYzvoqphFpfKBnaoo17mi_6bhiLUhQJasBVohftljPWeru0WXReUC0CgziszOkrBYwUhCJS8EbdneWIj4iaZOwTe1g_lvdCLv1wIl8PFdY8ZPMYNb9UPa3lg2VKjMZPjWLpz1tYug1nqFr6Wlhqs9eSaG4WuOIGZm6RIOIQH4QOcy_IeTLOjcOJnVo_NdS-nCpQexoQw3R8yxsEezLHumRJfgBWwDkUS72Dv5-De0VuWxnniHm1-GbDtHuvHwRNGJguBjm05vvKWLPZ8nMG37jZrCuWMuC2A2OIFPHiqTGkDjTRKgb5nlbCROEpyrxOhgDU9YRnJTvoXMmppXQIp8s3ZA=w923-h296-no?authuser=0)

There are three alternative models, called aliengo, laikago and a1. We studied the mobility of the joints of each model, the number of joints and other factors. These factors may affect our later simulations. We even considered copyright issues for the model. After comprehensive consideration, the third model in the figure, namely a1, was selected as the quadruped robot model to be used in our project. The model has a total of 12 movable joints. It can perform various actions very well. And its structure is relatively simple, it is very suitable for us beginners to get started. In addition, these three models are all models included in pybullet, which can be found in the data folder. This also saves us the step of querying the detailed data of the model.


![attemptcode](https://lh3.googleusercontent.com/kIv3ypfZ7DTq6eB10aTu20_cq6RxzcMJKsJ7WQ4qAmvSVs7QDnP3G3MZnOG9Y8dwKdQxW2zjIvgulwnEfYTmmQmzuAdTBtKkUedczt6epdNFpCS8Ytt0C94pQVqstqMMR4QpWQhc6ToL5tZy49Dcpt6XGyWMJ7E5wlgm4gp4bHPVoUviJQoF8wFFnisy6SVXWE42XoX2O4HQcXZK37IB6RIYSx50SJ3G2Z8yI4lhM29au5QNFQxbPY7D-a6x__mLSTHWjJfcLqIUlGXPOnKsIpTxvhjICDOn0hUAWroR_IMPDFkMvZDTGDy_TQH2N1j-jyXvJV5gQrZNfAfeGaAL0k4920iUZO9LKyoTXN_L-yqgkKH7-kQYGRq1TzP-WsgD1FGONS0HUy_NW2MS1a_lFTcprAV8xgMLXb4CqRiZ5iWcNlv9rW0bcC46obtfuBuuh7U4ou42lGcVCjAMINVBfGlOdUAxQeJ6D19pol5U_uCcui2M0IhtsNDeiolNo4BlbJXqyJYL61U2tW16nSLEGOong_va573OgWCwnqUK1mlJwxtk_1v87hj3kR0FSsrnFVU3oSqlTsWaHo_u4ixbqvPNEutZXWFRxI0OXXoEgsb3Ifco4KTt1uFC6xF1kCZuRM7Q5LAaZVqw9mlSYUZsG5qD80urQE3WhvT_yfo67mAKHHY5DieXwyKm61btqLQsLF5VibVlxd1cYcQP6SO7nw=w837-h254-no?authuser=0)

The code in the figure is the basic operation code provided by pybullet. With these few lines of code, we set the gravity, floor, and simulation rate of the simulated environment. Through the last line of the code, we successfully imported the quadruped robot a1 into the simulation environment.

![testcode](https://lh3.googleusercontent.com/B5x2nzHbCVxKBKDn6bsara-1CY7W_Bd2407M9ZlTAzMHeZgateW-lpeJFWGzHfEyKrOhF784juN9snAbUy5_eAPldvG8phFWStxAAeONBcs1kXqB_XO3T7v67JJ3jUBEypIci2DxRMYrAlH8s5kU1HgV1e05rEq73kF4B4THChPBvCaTNXHe1nGzxCuizJ9sDM-XxeAIPPz0RAc5xJZCuRqg1LufBky6R-X7d7Aue6Bs1FMBbYsg1pT6dRbvMqWgi1GJl63Myi9C8fmSlqx1fm3_JsZkpLLGgCPZhh4W-V-oufPy9VoVLh1wiGQLmIPVdOEC-89SGYgjzEki7QbsPAx0mNmO_q8d5F97RAV3BkzPzBlWeXEQphIofW4DRa2ZM-QpIYW412NlLtS4cR1y6lMrLgjsBQh_WqJPQv7kMne82hZGFwqw-dEiAm--g2mWufhGSVgTfGBeShQBhDVjQnKwpIXiLyibS705rovmQEkmiXJkez2X-Exuo8jHDcl9OCcLxCLbPFGbi0OC9JpFo_2FrffPA6QjSnPJZTsY8dobcgIALRbQOr8CjuWuIDvrYGJwQAJOVErr-lrby_Z1X-KBc56t17VIz_Tc8MyhMp1aXP9PZ8o0BknVcbLjaJkXqQ4wnVjT5WJ99WiwU5Y5b43fkWj13bYku-zoUaM7pS-VRnkQOsOjtONtThdJ6oyww46Rywt1vRh1KdJQ0rn3WA=w732-h76-no?authuser=0)

Through the code in the picture above, we control the robot's joint number 4 to lift forward. The result of the code execution is shown in the video below. It can be seen that the robot's right front leg is successfully raised forward.

![test video](https://lh3.googleusercontent.com/xyP034VTV3pJZLGqtWUDrIqR7sZsKOPWEEQqqwt2svKDfRCd_fSy8Tye4QngtbzovoZVT7aYtw3M3DPezHty9bCaxxim1bNwT7oUEPobowAncksZL3UC-1PzMOJ-UasePIbQLgy7udW83P46jsLVw3o9GgpEQtk7ldKEHwMLPNd5nxJZuznv62kuKFjX1rcfQMcKqgUs4dmP3XGTkgJXMClPlCIK7LL7E-hzU8BP07tbXoCAzPddV9cEqOSd_y2DWYJMzPrMPYPnoWdOc76D1fV91OVG_K57l2_XrzYLcrzIE4hgwpMG3QqqSlUDYUVnBF_Z5QYDWu2-yUz06sto3d26GruHc3_I2ayjAp0SPIYXaB_r31qV23sF-ySUL0x3oaGDfDJ9O_ayBf8PNPSnbL0GTFGdp4Hz4fyOH8AwhXBoyca9QDvwMaCOTm5LWAsDyO3zlt8hYfyLFbXN69XjhiQkIoKFypKE7dpxQzIaS7Z_t9I31v0zKackgM8iQu40u5E9VXOmhkF-QzlMqzZA2bi5fxpCxGZ_Kai55Igwm_DAKbFtXEa6hcfcyzaFPySDtGZOKf29pc-GTD8Wof7YKiJ-u2qCPkyU7Cwkc7TejK8nFNJcMhn_eA0P4p_idLt3EDlWh9LsVLB3dK3IFhrzuIVrmGH8sT0230mK2fkmLxdvvWixVNQbTkJWtvlXQs_oi8825dVaybEfHfZpqT3MNg=w889-h937-no?authuser=0)

After this, each member of the group actually tried it out. Everyone chooses different joints and different movement directions. The actual effect is similar to the video above, so we won't show it again.

![gantt chart](https://lh3.googleusercontent.com/vVodmGIPRfqAJTudWsHEh9XPVaW_lvZLYxjCunDowCLatLUu6jOF1mG0HUWgWXQa92vl79REELqMz4dlZmjRRo1v9VMJfqupE1OLetVh6aG3PlZc4CcH-Fqj-3oLk-5WYv4EODkNgR6JrQ0ARxLr-IX_2FooqDaU1jRvN8WsvZK4tKhBwqcOxX6kLDm7DOSgCRC3r6LTsKCSUGS2_R4qkBj2h-aB7qCW4ONrhXCbH4tqMaG3x6159MS6SBYFEA9HS8ZYvezbHz6UDh8ugzAXKSc2nvUJD0Yv8mt9WV22UmMNd5MbYpYwgUUda_qDlHT0VHimUs02zSKEYncDsYrW_rA9yLVAh8xISsVNOLmRyTywekXvCONms9OOq0ejUuoxVurlkDQikX1sXnycWPaBBr1MYezqgr6wZ22l3lPFS71Gh-C0ZCGJMYzbROvcQKH7wWzyrlMEbxCid8KRQThYneTh-niSURckdlwfoAc5LNubnTJdFaUAN6QlaS3_uTCN56Ae6qnIlaaUGkSsAfoeVfVUJbgxqEpuitMKnY2Qs3dACDePGtv_Wxe1kYcxq25e3iXxDnnplzlZ3VEbfauJaRHXY-e6av8R0YVxyaK5U8-zdWuozZNI2OOHKH3mDJe_6D2N2xmiZXNVvk46DL9SgLHgInU7VgtsfX-lOQ_pzjkC7UoRdiTz0G6Z_anfxm12d30s-OSwK-mB7gqAR1L-tw=w1470-h830-no?authuser=0)

Following aa's suggestion, we designed a gantt chart. From this chart, we can clearly see the assignment of tasks to each team member. The relationship between the various tasks is also clear at a glance. This undoubtedly makes it easier for us to control the progress of the project.



## Week 2 summary

 Learned read–eval–print loop (REPL), also termed an interactive toplevel or language shell, is a simple interactive computer programming environment that takes single user inputs, executes them, and returns the result to the user; a program written in a REPL environment is executed piecewise.
![vscode](https://i0.hdslb.com/bfs/album/8f32bd063ab801187c59bf7b99b4fe558603906d.png)
We completed the gantt chart task assigned by aa, which has a high positive significance for our work arrangement. Additionally, we selected a quadruped robot model for the simulation. Second, we configured the environment in the simulation software, such as gravity, floor, etc. In the end, the team members all succeeded in moving one of the robot's joints. This was a successful start for us, with a theoretical understanding of how to manipulate the joints to accomplish the simulated motion. Because an action is completed by the simultaneous motion of multiple joints, what we need to do next is to analyze how to call multiple joints at the same time to complete the simulation of the action.
