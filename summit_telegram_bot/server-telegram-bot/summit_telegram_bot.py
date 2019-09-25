#!/usr/bin/env python
# -*- coding: utf-8 -*-

from telegram.ext import Updater, CommandHandler
import logging
import sys

# Enable logging
logging.basicConfig(format='%(asctime)s-%(name)s-%(levelname)s-%(message)s',
                    level=logging.INFO)
logger = logging.getLogger(__name__)


def cmd_last_image(update, context):
    """Send the last image sent by the robot."""
    last_camera_img = '/var/www/robot-summit/uploads/last_camera_img.png'
    chat_id = update.message.chat_id
    s = '<b>Fecha de última imagen</b>: xxx'
    context.bot.send_message(chat_id=chat_id, text=s,
                             parse_mode=telegram.ParseMode.HTML)
    context.bot.send_photo(chat_id=chat_id, photo=open(last_camera_img, 'rb'))


def help(update, context):
    """Send a message when the command /help is issued."""
    s = "Help: ..."
    context.bot.send_message(chat_id=update.message.chat_id, text=s)


def error(update, context):
    """Log Errors caused by Updates."""
    logger.warning('Update "%s" caused error "%s"', update, context.error)


def main():
    if (len(sys.argv) != 2):
        logger.error("Required argument is missing")
        return

    token_file = str(sys.argv[1])
    print "Using TOKEN file: " + token_file
    token = open(token_file, "r").read().rstrip()
    updater = Updater(token=token, use_context=True)
    dp = updater.dispatcher
    dp.add_handler(CommandHandler('last_image', cmd_last_image))
    dp.add_handler(CommandHandler('help', help))
    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    main()
