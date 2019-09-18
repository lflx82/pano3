#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif
uniform float time;
uniform int state;
uniform int qiehuan;
uniform sampler2D texture;
uniform sampler2D texture1;
uniform sampler2D texture2;
uniform sampler2D texture3;
varying vec2 v_texcoord;

//! [0]
void main()
{
  
     if(state==1)
      {
           	 if(qiehuan==0)
	{
  		gl_FragColor = texture2D(texture, v_texcoord);
       	}
           	 else
	{
 		gl_FragColor = texture2D(texture, v_texcoord)*time+texture2D(texture1, 				v_texcoord)*(1-time);
	}
            
     }
      else if(state==2)
      {
               gl_FragColor = texture2D(texture2, v_texcoord);
       }
        else if(state==3)
         {
                gl_FragColor = texture2D(texture3, v_texcoord);
         }
       else if(state==4)
         {   
	gl_FragColor =vec4(1.0,0.0,0.0,1.0f);
         }
     
    else if(state==5)
         { 
	gl_FragColor =vec4(0.0,1.0,0.0,1.0f);
         }
 else if(state==6)
         { 
	gl_FragColor =vec4(0.0,0.0,1.0,1.0f);
         }
}


