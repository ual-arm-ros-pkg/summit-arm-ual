#!/usr/bin/env python
# -*- coding: utf-8 -*-

from telegram.ext import Updater, CommandHandler, MessageHandler, filters
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
    update.message.reply_text(text=s, parse_mode='HTML')
    context.bot.send_photo(chat_id=chat_id, photo=open(last_camera_img, 'rb'))


def help(update, context):
    """Send a message when the command /help is issued."""
    s = "Entiendo los siguientes comandos:\n"
    s += "/last_image: Lo que veo con mi cámara.\n"
    update.message.reply_text(text=s)


def cmd_generic_msg(update, context):
    """Generic conversation msg."""
    s = "Usa /help para ver lista de comandos."
    update.message.reply_text(text=s)


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

    dp.add_handler(MessageHandler(filters.Filters.all, cmd_generic_msg), 1)

    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    main()
