"""PX4MessageFilter: Synchronize messages from multiple topics by closest-in-time matching.

The first topic in the constructor list is treated as the reference (expected to be the
least frequent). Each incoming message triggers a synchronization attempt across all
non-empty buffers; matched sets are discarded from the front of each deque on success.
"""

from rclpy.node import Node
from rclpy.qos import QoSProfile
from collections import deque
from typing import Callable, List, Dict, Any
from functools import partial


class PX4MessageFilter:
    """
    Eager multi-topic timestamp synchronizer.

    Every message arrival triggers a synchronization pass.  The reference topic is the
    first element of ``topics``; it should be the sparse/slow topic so that matching
    is meaningful.
    """

    def __init__(
        self,
        node: Node,
        tolerance_us: int,
        topics: List[str],
        types: List[Any],
        qosprofiles: List[QoSProfile],
        buffer_depths: List[int],
        callback: Callable,
    ):
        """
        Initialize the PX4MessageFilter.

        :param node: The ROS 2 node instance.
        :param tolerance_us: The synchronization tolerance in microseconds.
        :param topics: List of topic names to synchronize. The first topic is expected to be the least frequent one and will be used as the reference for synchronization.
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
        self.tolerance_us = tolerance_us
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
                    partial(self._message_callback, topic=topic),
                    qosprofile,
                )
            )

    def _message_callback(self, msg: Any, topic: str):
        """
        Callback for incoming messages. Adds the message to the buffer.

        :param msg: The incoming message.
        :param topic: The topic the message was received on.
        :param msg_type: The type of the incoming message.
        """
        self.buffers[(topic)].append(msg)
        self._try_synchronize()

    def _try_synchronize(self):
        """
        Attempt to synchronize messages based on their timestamps.
        """
        # Get the reference topic (the first one in the list)
        reference_topic = list(self.buffers.keys())[0]
        ref_buffer = self.buffers[reference_topic]
        if not ref_buffer:
            return  # No messages in the reference buffer

        while ref_buffer:
            ref_msg = ref_buffer[0]  # Get the oldest message from the reference buffer
            ref_t = self._extract_timestamp(ref_msg)

            latest_time = max(
                self._extract_timestamp(buf[-1]) for buf in self.buffers.values() if buf
            )
            # wait for future messages if the latest message is withing the tolerance, otherwise we can discard this reference message and try the next one
            if latest_time - ref_t < self.tolerance_us:
                return

            matched_msgs = {reference_topic: ref_msg}
            all_matched = True
            for topic in self.topics[1:]:  # Check other topics
                buffer = self.buffers[topic]
                if not buffer:
                    all_matched = False
                    return  # No messages in this buffer, cannot synchronize
                while len(buffer) >= 2:
                    t0 = self._extract_timestamp(buffer[0])
                    t1 = self._extract_timestamp(buffer[1])
                    if abs(ref_t - t0) >= abs(ref_t - t1):
                        buffer.popleft()  # Discard the older message
                    else:
                        break  # The current message is closer, keep it
                candidate_msg = buffer[0]
                dt = abs(ref_t - self._extract_timestamp(candidate_msg))
                if dt <= self.tolerance_us:
                    matched_msgs[topic] = candidate_msg
                else:
                    ref_buffer.popleft()
                    all_matched = False
                    break  # This message is outside the tolerance, cannot synchronize
            if all_matched:
                self.callback(*matched_msgs.values())
                ref_buffer.popleft()
                continue  # Immediately try next synchronization
            break  # Only attempt to synchronize with the oldest reference message

    def _extract_timestamp(self, msg: Any) -> int:
        """
        Extract the timestamp from a message.

        :param msg: The message to extract the timestamp from.
        :return: The timestamp as an integer.
        """
        if hasattr(
            msg, "timestamp_sample"
        ):  # For px4_msgs. Attention that this is in microseconds
            return msg.timestamp_sample
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
