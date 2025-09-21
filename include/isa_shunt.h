/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



/* This library supports the ISA Scale IVT Modular current/voltage sensor
 * device. These devices measure current, up to three voltages, and provide
 * temperature compensation. This library was written by Jack Rickard of EVtv -
 * http://www.evtv.me copyright 2016. You are licensed to use this library for
 * any purpose, commercial or private, without restriction.
 */



    static void initialize();
    static void sendSTORE();
    static void STOP();
    static void START();
	
    static void handle521(uint32_t data[2]);
    static void handle522(uint32_t data[2]);
    static void handle523(uint32_t data[2]);
    static void handle524(uint32_t data[2]);
    static void handle525(uint32_t data[2]);
    static void handle526(uint32_t data[2]);
    static void handle527(uint32_t data[2]);
    static void handle528(uint32_t data[2]);



