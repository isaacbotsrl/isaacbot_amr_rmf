// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Mechatronics Academy orange (from the former logo) on a near-black app bar. */
import { createTheme } from '@mui/material';

const ORANGE = '#ffaa00';
const BAR = '#1d1d1b';

export const isaacbotTheme = createTheme({
  palette: {
    primary: { main: ORANGE, contrastText: '#000' },
    secondary: { main: ORANGE, contrastText: '#000' },
  },
  components: {
    MuiAppBar: {
      styleOverrides: {
        // The app bar takes primary by default; keep it dark so the white IsaacBot logo stands out.
        root: { backgroundColor: BAR, color: '#fff' },
      },
    },
  },
});
