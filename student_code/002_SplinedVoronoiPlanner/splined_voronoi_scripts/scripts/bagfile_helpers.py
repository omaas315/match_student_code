import rosbag


def get_contents_from_bagfile(filename: str):
    bag = rosbag.Bag(filename)
    bag_contents = {}
    for topic, msg, t in bag.read_messages(topics=[]):
        # assuming there is only one message per topic!
        bag_contents[topic] = msg
    bag.close()
    return bag_contents
