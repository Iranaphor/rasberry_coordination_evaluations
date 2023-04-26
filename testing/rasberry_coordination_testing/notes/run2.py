we need an alternative

we should be able to log agent positions within a database

when render the map for a specific time period, showing a ghost trail of each robot

since virtual robots publish their current edge and current node as they go along,
all we need to do is log it all into the db


for agent_id in agent_list:
    sub(String, f'/{agent_id}/current_edge', lambda msg: db.add(time.now(), [agent_id, msg.data]), 1)
    sub(String, f'/{agent_id}/current_node', lambda msg: db.add(time.now(), [agent_id, msg.data]), 1)


then we can make a seperate tool for displaying the data
perhaps a web app? perhaps something simpler
we include sessino/test details into the db and we can worry about that later
more info now means less retesting later

