# ESP32 'smart' kitchen scale

### TODO
1. There's a lot todo.
1. First is probably the documentation. :(
1. You can find out more at: [Hackaday.io](https://hackaday.io/project/164849-yet-another-smart-kitchen-scale).
1. Also see my revised [Google app script](https://gist.github.com/UriShX/e7182e1b6a151302f28b664d037c8b49).

### Linking the scale to your Google sheet
1. First, you'll need to setup your Google app script. You can see the github gist link above for a working script, which includes instructions. Make sure to read through the code and insert the required information before deploying (there are three separate places. Terrible design, I know.). Also, note that for the app script to work, **you have to push a new version with every change to the code**.
1. Change the file extension of the file called `google_script_stuff.#h` to `.h`.
1. Open your newly saved file, and replace the fields marked for replacement:
    1. `spreadSheetID` is String which identifies your specific spreeadsheet. You can get it by going to your spreadsheet and copying the uri string between *docs.google.com/spreadsheets/d/* and */edit*.
    1. `key` is a String, which will be your API key provided by google. The key string should be formatted as `key=*your_API_key*`.
    1. `SHEET_SCRIPT_URI` is a constant String (which I probably should make a `const char*` instead of a `#define`), which is a link to your published app script. You can get the link from the google app script online editor when deploying. The string should be of the form */macros/s/* YOUR_PUBLISHED_SCRIPT_ID */exec?* (without spaces, **and don't forget the '?' at the end!**).