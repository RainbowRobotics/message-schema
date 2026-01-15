class ZenohNoReply(Exception):
    pass

class ZenohTransportError(Exception):
    pass

class ZenohQueryException(Exception):
    pass

class ZenohReplyError(ZenohQueryException):
    pass
