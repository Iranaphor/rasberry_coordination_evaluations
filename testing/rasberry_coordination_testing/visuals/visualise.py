import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

import wandb

class Visualiser(Node):
    def __init__(self):
        super().__init__('executor')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map = dict()
        self.global_tmap_sub = self.create_subscription(String, '/topological_map_2', self.map_cb, qos)

        self.agents = dict()
        self.fleet_details_sub = self.create_subscription(AgentList, '/rasberry_coordination_core/agent_management/fleet_details', self.fleet_cb, qos)

    def map_cb(self, msg):
        # Log the raw map
        self.map = yaml.safe_load(msg.data)
        wandb.log({'raw_map': self.map})

        # Log the node list
        node_dict = {node["node"]["name"]: [node["node"]["pose"]["position"]["x"], node["node"]["pose"]["position"]["y"]] for node in self.map['nodes']}
        node_list = [[node["node"]["name"], node["node"]["pose"]["position"]["x"], node["node"]["pose"]["position"]["y"]] for node in self.map['nodes']]
        node_table = wandb.Table(data=node_list, columns=['node_name', 'x', 'y'])
        wandb.log({'node_table': node_table})

        # Log the edge list
        edge_list = []
        for node in self.map['nodes']:
            for edge in node["node"]["edges"]:
                edge_list.append([edge["edge_id"],
                                  node["node"]["name"],
                                  edge["node"],
                                  node_dict[node["node"]["name"]][0],
                                  node_dict[node["node"]["name"]][1],
                                  node_dict[edge["node"]][0],
                                  node_dict[edge["node"]][1]])
        edge_table = wandb.Table(data=edge_list, columns=['edge_name', 'start_name', 'end_name', 'start_x', 'start_y', 'end_x', 'end_y'])
        wandb.log({'edge_table': edge_table})

    def fleet_cb(self, a):
        # Loop through each agent with a changed location
        for a in msg.list:
            if (self.agents[a.id].location == a.location): continue

            # Save their new location
            l = a.location
            wandb.log(f'{a.agent_id}_location': [a.location.current_node,
                                                 a.location.current_edge,
                                                 a.location.closest_node,
                                                 a.location.closest_edge]})
            self.agents[a.id] = a


def main(args=None):
    rclpy.init(args=args)
    wandb.init(project="test9")

    Vi = Visualiser()
    rclpy.spin_once(Vi, timeout_sec=5)
    wandb.finish()
    return Vi

if __name__ == '__main__':
    Vi = main()

















# Test 2
#x_values = [1,2,3,4]
#y_values = [6,7,8,9]
#data = [[x, y] for (x, y) in zip(x_values, y_values)]
#table = wandb.Table(data=data, columns = ["x", "y"])
#line = wandb.plot.line(table, x="x", y="y", title="Cu")
#wandb.log({"topological_map_2": line})


# Test 3
#table = wandb.Table(data=[[3,4,'a','#FF0000'],[0,1,'a','#00FF00'],[7,6,'b','#0000FF']], columns=['x','y','name','colour'])
#lines = wandb.plot.line_series(xs=xs, ys=ys, keys=ns, title=self.map['metric_map'], xname='')

# Test 4
#wandb.log({'xs':sum(xs,[]), 'ys':sum(ys,[]), 'ns':sum(ns,[])})





#wandb.init(project="test")
#
#lines = wandb.plot.line_series(
#           xs=[[0, 1], [3, 4]],
#           ys=[[0, 1], [2, 4]],
#           keys=["metric Y", "metric Z"],
#           title="Two Random Metrics",
#           xname="x units")
#
#wandb.log({"my_custom_id" : lines})
#
#wandb.finish()

"""
this file is intended to subscribe to the tmap and the output from coordinator
then to generate images/plots which can be published to WandB to show routes generated over time

we should start be seeing what dashboards are possible with the tool
then we can decide if we want to send data to generate plots, or send images
we may also find an alternative tool which works more to our advantage

we also need to begin looking into storing our information more cohesively
perhaps having some database, to record our trials

1. Identify a tool for data analysis
2. Identify a tool for data storage



```
>>> a = [6, 2, 4, 7, 9]
>>> b = [8, 3, 6, 7, 3]
>>> c = ['a', 'b', 'c', 'd', 'e']
>>> z = [[a[i],b[i]] for i in [0,1,2,3,4]]

>>> wandb.init(project="test")
>>> table = wandb.Table(data=z, columns=["aa", "bb"])
>>> s = wandb.plot.scatter(table, x='aa', y='bb', title='scatterplotnew')
>>> wandb.log({'scatter_1':s})
>>> wandb.finish()
```

We start tomorrow, by attempting to generate a table render of the full tmap

"""














        ## If logging the map, log this here
        #if wandb.run.name.endswith('-1'):
        #    self.log_map(node_dict)
        #    print('Logging of map complete. Use Ctrl+C to exit!')

#        # Otherwise log a given route
#        N = ['WayPoint141', 'WayPoint140', 'WayPoint74', 'WayPoint66', 'WayPoint56', 'WayPoint63', 'r10-ca', 'r10-cb']
#        d1 = self.log_table(node_dict, N, '00FFFF')
#        print('Logging of route complete...')

#        N = ['r1-cz', 'r1-cy', 'r1-c0', 'r1-cb', 'r1-ca', 'WayPoint67', 'r1.5-ca', 'WayPoint144']
#        d2 = self.log_table(node_dict, N, '0000FF')
#        print('Logging of route complete. Use Ctrl+C to exit!')
#
#        # Join the two tables
#        table = wandb.Table(data=d1+d2, columns=['x', 'y', 'colour'])
#        wandb.log({"route_tables_joined": table})
#
#        # Publish custom vega spec
#        plot = wandb.plot_table(vega_spec_name="iranaphor/topologicalmap",
#                                data_table = table,
#                                fields = {"value" : "score",  "title" : "Plantae prediction scores"})
## ['x','y','colour'])
#        wandb.log({"route_tables_ultra": plot})
#        print("plus ULTRA")
#
#
#    def log_table(self, node_dict, node_list, colour):
#        # Define the route
#        data = [[node_dict[n][0], node_dict[n][1], colour] for i,n in enumerate(node_list)]
#        table = wandb.Table(data=data, columns=['x', 'y', 'colour'])
#        wandb.log({"route_table": table})
#        return data
#
#
#    def log_map(self, node_dict):
#        # Identify edges to include
#        edge_list = sum([[[node["node"]["name"],e["node"]] for e in node["node"]["edges"]]
#                    for node in self.map['nodes']],[])
#        edges = [list(i) for i in set(tuple(i) for i in edge_list)]
#
#        # Identify the components
#        xs = [ [node_dict[e[0]][0],node_dict[e[1]][0]] for e in edges]
#        ys = [ [node_dict[e[0]][1],node_dict[e[1]][1]] for e in edges]
#        ns = [ f"{e[0]}_{e[1]}" for e in edges]
#
#        # Construct and publish the wrapper
#        lines = wandb.plot.line_series(
#                   xs=xs,
#                   ys=ys,
#                   keys=ns,
#                   title=self.map['metric_map'],
#                   xname='')
#        wandb.log({"topological_map": lines})
#
#
#
#    def log_route(self, node_dict, node_list):
#        # Define the route
#        route = [[node_list[i],node_list[i+1]] for i in range(len(node_list)-1)]
#
#        # Identify the components
#        print('Logging of route.')
#        xs = [ [node_dict[e[0]][0],node_dict[e[1]][0]] for e in route]
#        ys = [ [node_dict[e[0]][1],node_dict[e[1]][1]] for e in route]
#        ns = [ f"{e[0]}_{e[1]}" for e in route]
#
#        # Construct and publish the wrapper
#        lines = wandb.plot.line_series(
#                   xs=xs,
#                   ys=ys,
#                   keys=ns,
#                   title=self.map['metric_map'],
#                   xname='')
#        wandb.log({"topological_map": lines})
#

