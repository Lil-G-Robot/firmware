    #[task(local = [imu], priority = 1)]
    async fn imu_test(ctx: imu_test::Context) {
        let imu = ctx.local.imu;

        loop {
            let acc_data = imu.read_acc().unwrap();
            let gyro_data = imu.read_gyro().unwrap();
            info!("\n\
                Accel Data (g): X = {}, Y = {}, Z = {}\n\
                Gyro Data (dps): X = {}, Y = {}, Z = {}",
            acc_data[0],
            acc_data[1],
            acc_data[2],
            gyro_data[0],
            gyro_data[1],
            gyro_data[2],
            );

            // delay
            Mono::delay(50.millis()).await;
        }

    }
