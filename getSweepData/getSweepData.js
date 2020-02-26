var iframe;

function onLoad() {
    iframe = document.getElementById('showcase_iframe');
    try {
        window.MP_SDK.connect(
            iframe, // Obtained earlier
            '***REMOVED***', // Your API key
            '3.2' // SDK version you are using
            // Use the latest version you can for your app
        )
            .then(loadedShowcaseHandler)
            .catch(handleError);
    }
    catch (e) {
        console.error(e);
    }
};

function loadedShowcaseHandler(mpSdk)
{
    mpSdk.Model.getData()
        .then(function (model) {
            console.log('Model sid:' + model.sid);
            console.log(JSON.stringify(model.sweeps, null, 2));
            //sweep = model.sweeps[0];
            //console.log('Sweep position:' + JSON.stringify(sweep.position));
            //console.log('Sweep rotation:' + JSON.stringify(sweep.rotation));
        })
        .catch(function (error) {
            console.log('Model cannot be retrieved');
            console.error(error);
        });
};

function handleError(error)
{
    console.error(error);
}