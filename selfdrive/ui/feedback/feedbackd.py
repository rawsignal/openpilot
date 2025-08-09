#!/usr/bin/env python3
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from cereal import custom
from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER

FEEDBACK_MAX_DURATION = 10.0
ButtonTypeSP = custom.CarStateSP.ButtonEvent.Type


def main():
  params = Params()
  pm = messaging.PubMaster(['userBookmark', 'audioFeedback'])
  sm = messaging.SubMaster(['rawAudioData', 'bookmarkButton', 'carStateSP'])
  should_record_audio = False
  block_num = 0
  waiting_for_release = False
  early_stop_triggered = False

  while True:
    sm.update()
    should_send_bookmark = False
    custom_button_mapping = params.get("SteeringCustomButtonMapping")

    if custom_button_mapping > 0 and sm.updated['carStateSP']:
      for be in sm['carStateSP'].buttonEvents:
        if be.type == ButtonTypeSP.customButton:
          if be.pressed:
            if not should_record_audio:
              if custom_button_mapping == 2:  # Start recording on first press if toggle set
                should_record_audio = True
                block_num = 0
                waiting_for_release = False
                early_stop_triggered = False
                cloudlog.info("Custom button pressed - starting 10-second audio feedback")
              else:
                should_send_bookmark = True  # immediately send bookmark if toggle false
                cloudlog.info("Custom button pressed - bookmarking")
            elif should_record_audio and not waiting_for_release:  # Wait for release of second press to stop recording early
              waiting_for_release = True
          elif waiting_for_release:  # Second press released
            waiting_for_release = False
            early_stop_triggered = True
            cloudlog.info("Custom button released - ending recording early")

    if should_record_audio and sm.updated['rawAudioData']:
      raw_audio = sm['rawAudioData']
      msg = messaging.new_message('audioFeedback', valid=True)
      msg.audioFeedback.audio.data = raw_audio.data
      msg.audioFeedback.audio.sampleRate = raw_audio.sampleRate
      msg.audioFeedback.blockNum = block_num
      block_num += 1
      if (block_num * SAMPLE_BUFFER / SAMPLE_RATE) >= FEEDBACK_MAX_DURATION or early_stop_triggered:  # Check for timeout or early stop
        should_send_bookmark = True  # send bookmark at end of audio segment
        should_record_audio = False
        early_stop_triggered = False
        cloudlog.info("10-second recording completed or second button press - stopping audio feedback")
      pm.send('audioFeedback', msg)

    if sm.updated['bookmarkButton']:
      cloudlog.info("Bookmark button pressed!")
      should_send_bookmark = True

    if should_send_bookmark:
      cloudlog.info("Sending bookmark")
      msg = messaging.new_message('userBookmark', valid=True)
      pm.send('userBookmark', msg)


if __name__ == '__main__':
  main()
