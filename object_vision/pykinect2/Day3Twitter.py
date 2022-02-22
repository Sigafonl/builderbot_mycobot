from twython import Twython
CONSUMER_KEY = ""
CONSUMER_SECRET = ""
ACCESS_TOKEN = ""
ACCESS_TOKEN_SECRET = ""
twitter = Twython(CONSUMER_KEY,
                  CONSUMER_SECRET,
                  ACCESS_TOKEN,
                  ACCESS_TOKEN_SECRET)


for status in twitter.search(q='"data science"')["statuses"]:
    user = status["user"]["screen_name"].encode('utf-8')
    text = status["text"]
    print(f"{user}: {text}\n")


from twython import TwythonStreamer
# Appending data to a global variable is pretty poor form
# but it makes the example much simpler
tweets = []

class MyStreamer(TwythonStreamer):
    def on_success(self, data):
       """        What do we do when Twitter sends us data?        Here data will be a Python dict representing a tweet.
       """
       # We only want to collect English-language tweets
       if data.get('lang') == 'en':
                tweets.append(data)
                print("received tweet #{"+str(len(tweets))+"}")
       # Stop when we've collected enough
       if len(tweets) >= 100:            self.disconnect()

    def on_error(self, status_code, data):
        print(status_code, data)
        self.disconnect()


stream = MyStreamer(CONSUMER_KEY, CONSUMER_SECRET,
                                 ACCESS_TOKEN, ACCESS_TOKEN_SECRET)

# starts consuming public statuses that contain the keyword ‘data’

stream.statuses.filter(track='data')             # we only care about “data”

print(tweets[0])     # print one example
