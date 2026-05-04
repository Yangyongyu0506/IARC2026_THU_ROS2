"""PX4MessageClamper: For each reference message find the straddling pair of messages in every other topic.

Useful when you need to interpolate a high-frequency signal around a slow reference sample.
The first topic is the reference; every other topic must accumulate at least two messages
that bracket (t0 <= ref_t <= t1) the reference timestamp before a callback fires.
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile
from collections import deque
from typing import Callable, List, Dict, Any
from functools import partial


class PX4MessageClamper:
    """
    Clamp a reference message between the two closest messages of each secondary topic.

    Reference topic callbacks trigger clamping.  All non-reference buffers must contain
    a pair whose timestamps enclose the reference timestamp; otherwise the reference
    message waits (or is dropped if newer secondary messages already passed it).
    """

    def __init__(
        self,
        node: Node,
        topics: List[str],
        types: List[Any],
        qosprofiles: List[QoSProfile],
        buffer_depths: List[int],
        callback: Callable,
    ):
        """
        Initialize the PX4MessageClamper.

        :param node: The ROS 2 node instance.
        :param topics: List of topic names to synchronize. The first topic is expected to be the reference topic to be clamped.
        :param types: List of message types for each topic.
        :param qosprofiles: List of QoS profiles for each topic.
        :param buffer_depths: List of buffer depths for each topic.
        :param callback: Function to call with synchronized messages.
        """
        assert len(topics) == len(types), (
            "Topics and types lists must be of the same length."
        )
        assert len(topics) == len(qosprofiles), (
            "Topics and QoS profiles lists must be of the same length."
        )
        assert len(topics) == len(buffer_depths), (
            "Topics and buffer depths lists must be of the same length."
        )
        self.topics = topics
        self.callback = callback
        self.buffers: Dict[str, deque] = {
            topic: deque(maxlen=buffer_depth)
            for topic, buffer_depth in zip(topics, buffer_depths)
        }
        self.subscriptions = []

        # Create subscriptions for each topic
        for id, (topic, msg_type, qosprofile) in enumerate(
            zip(topics, types, qosprofiles)
        ):
            self.subscriptions.append(
                node.create_subscription(
                    msg_type,
                    topic,
                    partial(
                        self._ref_message_callback
                        if id == 0
                        else self._ord_message_callback,
                        topic=topic,
                    ),
                    qosprofile,
                )
            )

    def _ord_message_callback(self, msg: Any, topic: str):
        """
        Callback for incoming messages. Adds the message to the buffer.

        :param msg: The incoming message.
        :param topic: The topic the message was received on.
        :param msg_type: The type of the incoming message.
        """
        self.buffers[(topic)].append(msg)

    def _ref_message_callback(self, msg: Any, topic: str):
        """
        Callback for incoming reference messages. Adds the message to the buffer and clamps it.

        :param msg: The incoming message.
        :param topic: The topic the message was received on.
        :param msg_type: The type of the incoming message.
        """
        self.buffers[(topic)].append(msg)
        self._clamp_msg()

    def _clamp_msg(self):
        """
        Find the two closest messages in the other topics to the reference message and calls the callback with the reference message and the clamped message pairs. Attention that this function assumes that the timestamp of the messages are mundanely increasing.
        """
        # Get the reference topic (the first one, which is also the oldest one in the buffer queue)
        reference_topic = list(self.buffers.keys())[0]
        ref_buffer = self.buffers[reference_topic]
        while ref_buffer:
            ref_msg = ref_buffer[0]  # Get the oldest message from the reference buffer
            ref_t = self._extract_timestamp(ref_msg)
            matched_msgs = {reference_topic: ref_msg}
            all_matched = True
            for topic in self.topics[1:]:  # Check other topics
                buffer = self.buffers[topic]
                if len(buffer) < 2:
                    return  # No enough messages to find a pair, wait for more messages
                matched = False
                while len(buffer) >= 2:
                    t0 = self._extract_timestamp(buffer[0])
                    t1 = self._extract_timestamp(buffer[1])
                    if t0 <= t1 < ref_t:
                        buffer.popleft()  # Discard the older message
                        continue
                    elif ref_t < t0 <= t1:
                        ref_buffer.popleft()  # Cannot match, discard the current reference topic
                        all_matched = False
                        break
                    elif t0 <= ref_t <= t1:  # Matched topic pair found
                        candidate_msg_pair = (buffer[0], buffer[1])
                        matched_msgs[topic] = candidate_msg_pair
                        matched = True
                        break
                    else:
                        return  # Cannot match, wait for more messages
                if not matched:  # THe valid length of the buffer is less than 2, wait for more messages
                    all_matched = False
                    break
            if all_matched:
                self.callback(*matched_msgs.values())
                ref_buffer.popleft()  # Remove the reference message after processing

    def _extract_timestamp(self, msg: Any) -> int:
        """
        Extract the timestamp from a message.

        :param msg: The message to extract the timestamp from.
        :return: The timestamp as an integer.
        """
        if hasattr(
            msg, "timestamp"
        ):  # For px4_msgs. Attention that this is in microseconds
            return msg.timestamp
        elif hasattr(msg, "header") and hasattr(
            msg.header, "stamp"
        ):  # For ROS 2 built-in types
            return (
                int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec) // 1000
            )  # Convert to microseconds
        else:
            raise ValueError(
                "Message does not have a recognizable timestamp attribute."
            )
