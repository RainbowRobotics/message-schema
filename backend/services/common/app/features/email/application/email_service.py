class EmailService:
    """
    [Email 서비스]
    """
    def __init__(self):
        self.email_port = email_port

    async def send_email(self, email: EmailModel):
        """
        - email: EmailModel
        - return: None
        """
        await self.email_port.send_email(email)
