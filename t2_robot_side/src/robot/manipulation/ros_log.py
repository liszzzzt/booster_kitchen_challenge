"""Send Python `logging` records of the manipulation modules to the ROS logger, so they show up in the
agent log (Studio: Agent Runtime) next to the node's own messages."""
import logging


class _RosHandler(logging.Handler):
    def emit(self, record):
        try:
            from rclpy.logging import get_logger
            logger = get_logger(record.name)
            msg = self.format(record)
            if record.levelno >= logging.ERROR:
                logger.error(msg)
            elif record.levelno >= logging.WARNING:
                logger.warning(msg)
            else:
                logger.info(msg)
        except Exception:
            pass


def route(*names):
    for name in names:
        logger = logging.getLogger(name)
        if not any(isinstance(h, _RosHandler) for h in logger.handlers):
            logger.addHandler(_RosHandler())
        logger.setLevel(logging.INFO)
