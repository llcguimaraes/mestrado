   M S B A M L   `   `   `  ����  BodyBasics-WPF4    .Microsoft.Samples.Kinect.BodyBasics.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  Body Basics��5   	   $	��600��5   	   $	��700��6    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush��%�   &	        &	 #     &	 F     �� 5	   
   $ 	#ff6e6e6e��62   �� 5
   
   $ 	#ff52318f��64   �� 5   
   $ 	#ff00BCF2��62   5      .��5      � $��
10,0,10,50q�6    � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $��* �6   �� 5      $
��Auto �6   .+�5   
   (� $��1��6   $��Center=�6   .��5      �� $��UniformToFill=�63   ��6   �� *ImageSource+63   5      �� 5   
   -   5        $��	statusBar��5   ?   $��2��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   .@�5      �� ��6   �� *
StatusText+5      �� 5   
   -   5         ��Name$ flexao_extensao_ombro��5      $��Flexão/Extensão Ombro D��62   $
��Left=�6T   $��16��6o   $��-3,20,0,-30q�6{   $��2��6�   $	��Top=�6�   $	��158��6�   �� 5   
   -   5        $ abducao_ombro��5      $��Abdução Ombro D��6*   $
��Left=�6D   $��16��6_   $��157,20,0,-30q�6k   $��2��6�   $	��Top=�6�   $	��122��6�   �� 5   
   -   5        $ aducao_ombro��5      $��Adução Ombro D��6)   $
��Left=�6B   $��16��6]   $��279,20,0,-30q�6i   $��2��6   $	��Top=�6�   $	��114��6�   �� 5   
   -   5        $$ flexao_extensao_ombro_esquerdo��5      $��Flexão/Extensão Ombro E��6;   $
��Left=�6]   $��16��6x   $��-3,41,0,-51q�6�   $��2��6�   $	��Top=�6�   $	��158��6�   �� 5   
   -   5        $ abducao_ombro_esquerdo��5      $��Abdução Ombro E��63   $
��Left=�6M   $��16��6h   $��157,41,0,-51q�6t   $��2��6�   $	��Top=�6�   $	��122��6�   �� 5    
   -   5        $ aducao_ombro_esquerdo��5       $��Adução Ombro E��62   $
��Left=�6K   $��16��6f   $��279,41,0,-51q�6r   $��2��6�   $	��Top=�6�   $	��114��6�   �� 5!   
   -	   5        $  flexao_extensao_cotovelo_d��5!      $��Flexão/Extensão Cot D��67   $
��Left=�6W   $��16��6r   $��398,20,0,-30q�6~   $��2��6�   $	��Top=�6�   $	��141��6�   �� 5"   
   -
   5        $# pronacao_supinacao_cotovelo_d��5"      $"��Pronação/Supinação Cot ]��6:   $
��Left=�6]   $��16��6x   $��539,20,0,-30q�6�   $��2��6�   $	��Top=�6�   $	��124��6�   �� 5#   
   -   5        $  flexao_extensao_cotovelo_e��5#      $��Flexão/Extensão Cot E��67   $
��Left=�6W   $��16��6r   $��398,41,0,-51q�6~   $��2��6�   $	��Top=�6�   $	��141��6�   �� 5$   
   -   5        $# pronacao_supinacao_cotovelo_e��5$      $"��Pronação/Supinação Cot E��6:   $
��Left=�6]   $��16��6x   $��539,41,0,-51q�6�   $��2��6�   $	��Top=�6�   $	��124��6�   5%      5&      