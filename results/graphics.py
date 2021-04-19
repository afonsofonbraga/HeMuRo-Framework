import re
import matplotlib.pyplot as plt
import numpy as np

def autolabel(rects, axss):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        axss.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')
                    
#hand = open('/home/robot/github/HeMuRo-Framework/logs/Logger131/Missions.txt')
hand = open('/Users/afonsofonbraga/github/HeMuRo-Framework/logs/Logger120/Missions.txt')
mission_owner = dict()
mission_executioner = dict()
mission_redirected = dict()
mission_lowbattery = dict()
mission_failure = dict()
mission_timeout = dict()
mission_null = dict()
charging_request = dict()
charging_station_count = dict()

for line in hand:
    line = line.rstrip()
    
    missions = re.findall('.*null$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            owner = words[1]
            
            mission_owner[owner] = mission_owner.get(owner, 0 ) + 1
# Creating a dicionary with all agents who completed at least one mission [agentsName] = Number of completed missions
    missions = re.findall('.*Complete$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            executioner = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            mission_executioner[executioner] = mission_executioner.get(executioner, 0) + 1

# Creating a dicionary with all agents who redirected at least one mission [agentsName] = Number of redirected missions
    missions = re.findall('\[.*\]\[(.*)\].*Redirected$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7 and words[2] == mission:
            executioner = words[2]
            
            mission_redirected[executioner] = mission_redirected.get(executioner, 0 ) + 1

# Creating a dicionary with all agents who redirected at least one misison due to lowbattery [agentsName] = Number of redirected missions
    missions = re.findall('.*LowBattery$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            executioner = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            mission_lowbattery[executioner] = mission_lowbattery.get(executioner, 0) + 1
            
# Creating a dicionary with all agents who redirected at least one misison due to timeout [agentsName] = Number of redirected missions
    missions = re.findall('.*Timeout$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            executioner = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            mission_timeout[executioner] = mission_timeout.get(executioner, 0) + 1

# Creating a dicionary with all agents who redirected at least one misison due to failure [agentsName] = Number of redirected missions
    missions = re.findall('.*Failure$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            executioner = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            mission_failure[executioner] = mission_failure.get(executioner, 0) + 1
            
# Creating a dicionary with all agents who Requested to Charge [agentsName] = Number of Charging requests
    missions = re.findall('.*ChargingRequested$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            owner = words[1]
            station = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            charging_request[owner] = charging_request.get(owner, 0) + 1

# Creating a dicionary with all agents who redirected at least one misison due to failure [agentsName] = Number of redirected missions
    missions = re.findall('.*ChargingCompleted$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            station = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            charging_station_count[station] = charging_station_count.get(station, 0) + 1
            
print("Mission Offered:")
print(mission_owner)
print("Mission Completed")
print(mission_executioner)
print("Mission Redirected")
print(mission_redirected)
print("Mission Redirected - LowBattery")
print(mission_lowbattery)
print("Mission Redirected - Timeout")
print(mission_timeout)
print("Mission Redirected - Failure")
print(mission_failure)
print("Charging Requests")
print(charging_request)
print("Charging Requests Completed for each station")
print(charging_station_count)

# Adding missing agents in each dictinary in order to plot correctly

for agent in list(mission_owner.keys()):
    charging_request[agent] = charging_request.get(agent, 0 )
for agent in list(charging_request.keys()):
    mission_owner[agent] = mission_owner.get(agent, 0 )

for agent in list(mission_redirected.keys()):
    mission_executioner[agent] = mission_executioner.get(agent, 0 )
for agent in list(mission_executioner.keys()):
    mission_redirected[agent] = mission_redirected.get(agent, 0 )
    
for agent in list(mission_executioner.keys()):
    mission_lowbattery[agent] = mission_lowbattery.get(agent, 0 )
for agent in list(mission_executioner.keys()):
        mission_timeout[agent] = mission_timeout.get(agent, 0 )
for agent in list(mission_executioner.keys()):
        mission_failure[agent] = mission_failure.get(agent, 0 )


# criando o primeiro grafico
label_onwer = list(mission_owner.keys())
ordered_owner = list(mission_owner.values())

#ordered_owner = list()
ordered_charging_request = list()
#criar na uma lista ordenada, com os valores de charging_station_count mas na ordem de mission_owner.keys()
for agent in label_onwer:
    ordered_charging_request.append(charging_request.get(agent))

label_executioner = list(mission_executioner.keys())
size_executioner = list(mission_executioner.values())
explode_executioner = [0] * len(size_executioner)

label_redirected = list(mission_executioner.keys())
ordered_redirected = list()
ordered_lowbattery = list()
ordered_failure = list()
ordered_timeout = list()

for agent in list(mission_redirected.keys()):
	ordered_redirected.append(mission_redirected.get(agent))
for agent in list(mission_redirected.keys()):
    ordered_lowbattery.append(mission_lowbattery.get(agent))
for agent in list(mission_redirected.keys()):
    ordered_failure.append(mission_failure.get(agent))
for agent in list(mission_redirected.keys()):
    ordered_timeout.append(mission_timeout.get(agent))


explode_redirected = [0] * len(ordered_redirected)

fig, axs = plt.subplots(2, 2, figsize=(11, 6))

x = np.arange(len(label_onwer))  # the label locations
width = 0.05  # the width of the bars

rects0 = axs[0,0].bar(x - width/2, ordered_owner, width, label='Mission')
rects01 = axs[0,0].bar(x + width/2, ordered_charging_request, width, label='Charging Requests')

axs[0,0].set(title= "Mission Owners")
axs[0,0].set_ylabel('Missions')
axs[0,0].set_xticks(x)
axs[0,0].set_xticklabels(label_onwer)
axs[0,0].legend()


axs[0,1].pie(size_executioner, explode=explode_executioner, labels=label_executioner, autopct='%1.1f%%', shadow=True, startangle=90)
axs[0,1].axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
axs[0,1].set(title= "Mission Executed")


x = np.arange(len(label_redirected))  # the label locations
width = 0.05  # the width of the bars

rects1 = axs[1,0].bar(x - width/2, size_executioner, width, label='Completed')
rects2 = axs[1,0].bar(x + width/2, ordered_redirected, width, label='Redirected')

# Add some text for labels, title and custom x-axis tick labels, etc.
axs[1,0].set_ylabel('Missions')
axs[1,0].set_title('Missions Completed and Redirected')
axs[1,0].set_xticks(x)
axs[1,0].set_xticklabels(label_executioner)
axs[1,0].legend()

#rects3 = axs[1,1].bar(x - width/2, ordered_redirected, width, label='Redirected')
rects4 = axs[1,1].bar(x - width, ordered_lowbattery, width, label='LowBattery')
rects5 = axs[1,1].bar(x, ordered_failure, width, label='Failure')
rects6 = axs[1,1].bar(x + width, ordered_timeout, width, label='Timeout')
 
# Add some text for labels, title and custom x-axis tick labels, etc.
axs[1,1].set_ylabel('Missions')
axs[1,1].set_title('Redirection Reasons')
axs[1,1].set_xticks(x)
axs[1,1].set_xticklabels(label_executioner)
axs[1,1].legend()


autolabel(rects0, axs[0,0])
autolabel(rects01, axs[0,0])
autolabel(rects1, axs[1,0])
autolabel(rects2, axs[1,0])
#autolabel(rects3, axs[1,1])
autolabel(rects4, axs[1,1])
autolabel(rects5, axs[1,1])
autolabel(rects6, axs[1,1])

fig.tight_layout()
plt.show()
