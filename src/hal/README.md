# HAL 
 avnt de lancer la hal les moteurs doivent etre corectement configurés

 depuis Tinymover studio renseigner le can ID ainsi que les parrametres de resistance, inductance et de courant.

pour ceci utilisez la commande [set_motor_config](https://tinymovr.readthedocs.io/en/latest/api/guide.html)


dans notre cas nous avons deux types de moteur :

MAD5010 200KV : 
{'R': 0.17272867262363434 <Unit('ohm')>,
'L': 5.0152262701885775e-05 <Unit('henry')>}

deuxiemme solution effetuer une calibration du moteur "tmx.calibrate()"

puis enregistrez les parrametres "tmx.save_config()"
