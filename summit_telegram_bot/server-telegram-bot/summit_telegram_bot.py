#!/usr/bin/python3
# -*- coding: utf-8 -*-

from telegram.ext import Updater, CommandHandler, MessageHandler, filters
from datetime import datetime
import logging
import os
import sys
from chatterbot import ChatBot
from chatterbot.trainers import ChatterBotCorpusTrainer

# Enable logging
logging.basicConfig(format='%(asctime)s-%(name)s-%(levelname)s-%(message)s',
                    level=logging.INFO)
logger = logging.getLogger(__name__)

# -- Configuration and constants --
UPLOADS_BASEDIR = '/var/www/robot-summit/uploads/'
FILENAME_CAMERA_IMG = 'last_camera_img.png'
FILENAME_BATTERY = 'last_battery.txt'
FILENAME_LIDAR = 'last_lidar.png'
FILENAME_MAP = 'last_map.png'
FILENAME_POSE = 'last_pose.txt'
# -------------------------------


# ChatBot:
myChatBot = ChatBot(
    'Summit',  # name

    storage_adapter='chatterbot.storage.SQLStorageAdapter',
    database_uri='sqlite:///summit-chatter-bot.sqlite3',

    logic_adapters=[
        {
            "import_path": "chatterbot.logic.BestMatch",
            "statement_comparison_function": "chatterbot.comparisons.levenshtein_distance",
            "maximum_similarity_threshold": 0.35,
            "default_response": "Perdona, no te he entendido bien. Usa /help para ver los comandos que entiendo."
        }
        #        {
        #            'import_path': 'chatterbot.logic.SpecificResponseAdapter',
        #            'input_text': 'Q',
        #            'output_text': 'A'
        #        }
    ],

    preprocessors=[
        'chatterbot.preprocessors.clean_whitespace'
    ]
)


def help(update, context):
    """Send a message when the command /help is issued."""
    s = "Entiendo los siguientes comandos:\n"
    s += "/show_battery: Mide el voltaje de mi batería.\n"
    s += "/show_camera: Lo que veo con mi cámara.\n"
    s += "/show_lidar: Lo que ve mi escáner láser.\n"
    s += "/show_map: El mapa de mi entorno.\n"
    s += "/show_pose: ¿Dónde estoy en el mapa?.\n"
    update.message.reply_text(text=s)


def reply_file_and_timestamp(update, context, filename, msg):
    modtime = os.stat(filename).st_mtime
    s = '<b>' + msg + '</b >: ' + \
        str(datetime.fromtimestamp(modtime))
    update.message.reply_text(text=s, parse_mode='HTML')
    context.bot.send_photo(chat_id=update.message.chat_id,
                           photo=open(filename, 'rb'))


def reply_txt_file_and_timestamp(update, context, filename, msg):
    modtime = os.stat(filename).st_mtime
    s = '<b>' + msg + '</b >: ' + \
        str(datetime.fromtimestamp(modtime)) + '\n'
    update.message.reply_text(text=s, parse_mode='HTML')
    update.message.reply_text(
        text=open(filename, 'r').read().rstrip(), parse_mode='HTML')


def cmd_last_image(update, context):
    """Send the last image sent by the robot."""
    reply_file_and_timestamp(
        update, context, UPLOADS_BASEDIR + FILENAME_CAMERA_IMG,
        'Fecha de última imagen')


def cmd_last_map(update, context):
    reply_file_and_timestamp(
        update, context, UPLOADS_BASEDIR + FILENAME_MAP,
        'Última actualización del mapa')


def cmd_last_lidar(update, context):
    reply_file_and_timestamp(
        update, context, UPLOADS_BASEDIR + FILENAME_LIDAR,
        'Fecha de último barrido LIDAR')


def cmd_last_pose(update, context):
    reply_txt_file_and_timestamp(
        update, context, UPLOADS_BASEDIR + FILENAME_POSE,
        'Última actualización de pose')


def cmd_last_battery(update, context):
    reply_txt_file_and_timestamp(
        update, context, UPLOADS_BASEDIR + FILENAME_BATTERY,
        'Última actualización de batería')


def cmd_generic_msg(update, context):
    """Generic conversation msg."""
    in_msg = update.message.text
    # if input was a commmand, stop further processing:
    if (in_msg[0] == '/'):
        return

    # Use the natural language engine to answer:
    bot_output = myChatBot.get_response(in_msg)
    update.message.reply_text(text=bot_output.text)


def error(update, context):
    """Log Errors caused by Updates."""
    s = 'Update "%s" caused error "%s"' % (update, context.error)
    logger.error(s)
    update.message.reply_text(text=s)


def main():
    if (len(sys.argv) != 3):
        logger.error(
            "Usage: summit_telegram_bot.py <TELEGRAM_API_TOKEN_FILE> <CHATBOT_TRAIN.yml>")
        return

    token_file = str(sys.argv[1])
    chatbot_train_file = str(sys.argv[2])

    trainer = ChatterBotCorpusTrainer(myChatBot)
    trainer.train(chatbot_train_file)

    print("Using TOKEN file: " + token_file)
    token = open(token_file, "r").read().rstrip()
    updater = Updater(token=token, use_context=True)
    dp = updater.dispatcher
    dp.add_handler(CommandHandler('show_camera', cmd_last_image))
    dp.add_handler(CommandHandler('show_map', cmd_last_map))
    dp.add_handler(CommandHandler('show_lidar', cmd_last_lidar))

    dp.add_handler(CommandHandler('show_battery', cmd_last_battery))
    dp.add_handler(CommandHandler('show_pose', cmd_last_pose))

    dp.add_handler(CommandHandler('help', help))

    dp.add_handler(MessageHandler(filters.Filters.all, cmd_generic_msg), 1)

    # log all errors
    dp.add_error_handler(error)

    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    main()
