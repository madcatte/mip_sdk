const net = require('net');
const mip_sdk = require('../../build/node/build/Debug/mip_sdk');

var cached_buffer = Buffer.alloc(0);
const client = new net.Socket();
client.connect(5000, '127.0.0.1', () => {

    mip_sdk.mip_interface_set_recv_callback((max_length) => {
        // Grab the number of bytes that we can return and then trim them from the buffer
        const end_index = (max_length < cached_buffer.length ? max_length : cached_buffer.length) - 1;
        const recv_buffer = cached_buffer.slice(0, end_index);
        cached_buffer = cached_buffer.slice(end_index + 1, cached_buffer.length - 1);
        return recv_buffer;
    });

    mip_sdk.mip_interface_set_send_callback((data) => {
        // Just send the data to the TCP socket
        client.write(data);
        return true;
    });

    client.on('data', (data) => {
        cached_buffer = Buffer.concat([cached_buffer, data]);
    });

    // Initialize the MIP interface
    const mip_interface = new mip_sdk.mip_interface();
    const buffer = mip_sdk.malloc_u8(1024);
    mip_sdk.mip_interface_init(mip_interface, buffer, 1024, 1000, 2000);

    // Just set the device to idle
    // TODO: This will block forever, meaning that the response will never be received
    const result = mip_sdk.mip_base_set_idle(mip_interface);
    console.log(mip_sdk.mip_cmd_result_to_string(result));

    // Shut down the TCP socket
    client.end();
})
