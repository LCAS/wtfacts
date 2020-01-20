# WTFACTS

Wavelet-based Temporal Forecasting Models Action Service for ROS

This ROS package manages the creation, query and modification of wavelet-based models for temporal series forecasting. This version is focused on binary series forecasting.

## Install

First, you will need to add the L-CAS repository to resolve some ROS dependencies. Please follow the instructions here: [Using the L-CAS repository](https://github.com/LCAS/rosdistro/wiki#using-the-l-cas-repository)

After that, you can clone this repository. For example, creating its own workspace:

```bash
mkdir -p ~/workspace/wtf/src
cd ~/workspace/wtf/src
git clone https://github.com/LCAS/wtfacts.git
```

And installilng the necessary dependencies:

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### Optional: installing Mongo and datasets

Our datasets are stored as mongo databases. You can download our datasets and add them to your local mongo server.

First, you will need to install first a mongo server:

```bash
sudo apt-get install mongodb-server
```

Download our datasets here:

- [L-CAS domotic dataset](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-domotic-sensors-dataset/)

- [ENRICHME domotic dataset](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/lace-house-domotic-sensors-dataset/)

And install them into your server:

```bash
mongorestore  --db LCAS1 /home/jhon/mongo/LCAS2016/dump/openhab/
mongorestore  --db LCAS2 /home/jhon/mongo/LCAS2017/dump/openhab/
mongorestore  --db ENRICHME /home/jhon/mongo/LACE2017/dump/openhab/
```

## Usage

As any action server, we need to fill in fields in the goal message to interact with the server.
Depending on the requested operation, some fields in the goal message will be necesary or not. This server supports 5 basic commands, by setting the field `operation` with the strings: [`bcreate` | `append` | `predict` | `delete` | `list`]

-  `bcreate`: creates a new binary wavelet [model] using the corresponding [values] and [timestamps]
-  `append`: appends [values] and [timestamps] to the provided [model], recalculating the model
-  `predict`: returns expected [values] at provided [model] and  [timestamps] 
-  `delete`: drops [model] from server
-  `list`: returns in [message] a list of available wavelet models

Reply to the `operation` will have 3 fields:

- success: operation result
- values : predicted values
- message: result description OR wavelet models list

## Example
See [wtf_client.py](https://github.com/LCAS/wtfacts/blob/master/scripts/wtf_client.py) for a full working client, also with connection to mongo databae.

## Reference

If you are considering using this work, please reference the following:

Fernandez-Carmona M. and Bellotto N. (2020) *“Wavelet-based Temporal Models of Human Activities for Anomaly Detection”.*